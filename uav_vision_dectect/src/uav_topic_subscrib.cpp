//
// Created by verse on 24-10-9.
//
#include "uav_topic_subscrib.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "preprocess.h"


#define USE_FP16
#define DEVICE 0
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000

using namespace std;
using namespace cv;

// 网络相关常量
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;



void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize)
{
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

UavTopicSubscrib::UavTopicSubscrib() : Node("uav_topic_subscrib")
{

    /***********************************局部跟踪器初始化***********************************/
    std::string init_model = "/home/verse/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_init";
    std::string update_model = "/home/verse/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_update";

    siam_tracker = new LightTrack(init_model.c_str(), update_model.c_str());


    /***********************************话题订阅***********************************/
    uav_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        10,
        std::bind(&UavTopicSubscrib::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "------------uav_topic_subscrib------------");


    // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


    // global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    //      "/px4_1/fmu/out/vehicle_global_position",
    //      qos,
    //      std::bind(&UavTopicSubscrib::global_position_callback, this, std::placeholders::_1)
    //  );


    float half_length = 3.62;   // 长度m
    float half_width  = 0.72;    // 宽度
    float height = 6;           // 高度


    uav_result_rect.x = -1;
    uav_result_rect.y = -1;
    uav_result_rect.width = -1;
    uav_result_rect.height = -1;

    // 定义无人机与xy平面共面
    objectPoints.push_back(cv::Point3f(0, 0, 0));                            // 点1: (0, 0, 0)
    objectPoints.push_back(cv::Point3f(0, half_length, 0));                  // 点2: (0, 3.62, 0)
    objectPoints.push_back(cv::Point3f(half_width, 0, 0));                   // 点3: (0.72, 0, 0)
    objectPoints.push_back(cv::Point3f(half_length, half_width, 0));         // 点4: (3.62, 0.72 , 0)

    Mat(objectPoints).convertTo(objPM, CV_32F);             //把三维点向量变成三维点矩阵

    uav_detect_result_publisher_ = this->create_publisher<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10);
    uav_detect_result_thread_ = std::thread(&UavTopicSubscrib::uav_detect_result_loop, this);

}



void UavTopicSubscrib::uav_detect_result_loop()
{
    rclcpp::WallRate loop_rate(30);

    while (rclcpp::ok())
    {
        pub_uav_result_rect.header = std_msgs::msg::Header();
        pub_uav_result_rect.x = uav_result_rect.x;
        pub_uav_result_rect.y = uav_result_rect.y;
        pub_uav_result_rect.width = uav_result_rect.width;
        pub_uav_result_rect.height = uav_result_rect.height;
        pub_uav_result_rect.depth = obj_depth;                      //添加深度信息，该深度相对于目标位置

        RCLCPP_INFO(this->get_logger(), "Detect_result: %d %d %d %d", uav_result_rect.x, uav_result_rect.x, uav_result_rect.width, uav_result_rect.height);

        
        uav_detect_result_publisher_->publish(pub_uav_result_rect);

        loop_rate.sleep();
    }
}



void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 使用cv_bridge将ROS图像消息转换为OpenCV图像
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }


    uav_camera_frame = cv_ptr->image;
    cv::Mat frame = cv_ptr->image;
    frame.copyTo(uav_camera_frame);   //用于其他调用


    cudaSetDevice(DEVICE);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string engine_name = "/home/verse/ros2_ws/src/uav_vision_dectect/model/yolov5/yolov5_little_uav.engine";
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "Failed to load engine file!" << std::endl;
    }
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);

    std::vector<char> trtModelStream(size);
    file.read(trtModelStream.data(), size);
    file.close();

    IRuntime* runtime = createInferRuntime(gLogger);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream.data(), size);
    IExecutionContext* context = engine->createExecutionContext();
    assert(context != nullptr);

    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    float* buffers[2];
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);

    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));

    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    uint8_t* img_host = nullptr;
    uint8_t* img_device = nullptr;
    CUDA_CHECK(cudaMallocHost(&img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
    CUDA_CHECK(cudaMalloc(&img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));


    float* buffer_idx = (float*)buffers[inputIndex];
    size_t size_image = frame.cols * frame.rows * 3;
    memcpy(img_host, frame.data, size_image);
    CUDA_CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
    preprocess_kernel_img(img_device, frame.cols, frame.rows, buffer_idx, INPUT_W, INPUT_H, stream);

    if(light_track_flag == 0)
    {
        doInference(*context, stream, (void**)buffers, prob, BATCH_SIZE);

        std::vector<Yolo::Detection> res;
        nms(res, prob, CONF_THRESH, NMS_THRESH);

        cv::Rect best_rect;
        float best_score = 0;
        for (const auto& detection : res) {
            if (detection.conf > best_score) {
                float bbox_array[4] = {detection.bbox[0], detection.bbox[1], detection.bbox[2], detection.bbox[3]};
                best_rect = get_rect(frame, bbox_array);
                best_score = detection.conf;
            }
        }

        Bbox box;
        uint8_t *img = frame.data;
        if (best_score > 0.3) 
        {
            // cv::rectangle(frame, best_rect, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(frame, "Global Track Success", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0x27, 0xC1, 0x36), 2);
            light_track_flag = 1;

            //跟踪器初始化
            this->trackWindow = best_rect;
            // std::cout << "x:" << best_rect.x << " y:" << best_rect.y << std::endl;
            std::cout << "Start track init ..." << std::endl;
            std::cout << "==========================" << std::endl;

            box.x0 = trackWindow.x;
            box.x1 = trackWindow.x + trackWindow.width;
            box.y0 = trackWindow.y;
            box.y1 = trackWindow.y + trackWindow.height;

            if( box.x0 >= frame.cols  ||
                ((box.x0 + (box.x1 - box.x0) / 2) >= frame.cols) ||
                ((box.y0 + (box.y1 - box.y0) / 2) >= frame.rows) ||
                box.y0 >= frame.rows)
            {
                RCLCPP_INFO(this->get_logger(), "light track error 1");


            }

            siam_tracker->init(img, box, frame.rows, frame.cols);
            std::cout << "==========================" << std::endl;
            std::cout << "Init done!" << std::endl;
            std::cout << std::endl;

            frame(trackWindow).copyTo(init_window);

        }else{
            uav_result_rect.x = -1;
            uav_result_rect.y = -1;
            uav_result_rect.width = -1;
            uav_result_rect.height = -1;
        }
    }


    if(light_track_flag == 1)
    {

        uint8_t * img_track = frame.data;

        if( siam_tracker->target_pos.x >= frame.cols  ||
            siam_tracker->target_pos.y >= frame.rows)
        {
            RCLCPP_INFO(this->get_logger(), "light track error 2");
        }

        siam_tracker->track(img_track);

        cv::Rect rect;
        cxy_wh_2_rect(siam_tracker->target_pos, siam_tracker->target_sz, rect);

        // Boundary judgment.
        cv::Mat track_window;
        if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= frame.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= frame.rows)
        {
            frame(rect).copyTo(track_window);

            if(siam_tracker->target_pos_change() == 0) {
                cv::putText(frame, "Local Track Success", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::rectangle(frame, rect, cv::Scalar(0x27, 0xC1, 0x36), 2);
                uav_result_rect = rect;

                std::vector<cv::Point2f> imagePoints;
                cv::Point2f topLeft(rect.x, rect.y); // 左上角
                cv::Point2f topRight(rect.x + rect.width, rect.y); // 右上角
                cv::Point2f bottomLeft(rect.x, rect.y + rect.height); // 左下角
                cv::Point2f bottomRight(rect.x + rect.width, rect.y + rect.height); // 右下角

                // 将四个点加入 vector 中
                imagePoints.clear();
                imagePoints.push_back(topLeft);
                imagePoints.push_back(topRight);
                imagePoints.push_back(bottomRight);
                imagePoints.push_back(bottomLeft);

                // 使用solvePnP函数计算相机的位姿
                solvePnP(objPM, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
                double rm[3][3];
                cv::Mat rotMat(3, 3, CV_64FC1, rm);
                Rodrigues(rvec, rotMat);  // 将旋转向量转换为旋转矩阵
                // 从旋转矩阵中提取欧拉角
                float theta_z = atan2(rm[1][0], rm[0][0]) * 57.2958;
                float theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;
                float theta_x = atan2(rm[2][1], rm[2][2]) * 57.2958;

                // 获取平移向量
                double tx = tvec.ptr<double>(0)[0];
                double ty = tvec.ptr<double>(0)[1];
                double tz = tvec.ptr<double>(0)[2];
                obj_depth = tz;


                // // 根据欧拉角调整平移向量
                // CodeRotateByZ(tx, ty, -1 * theta_z, tx, ty);
                // CodeRotateByY(tx, tz, -1 * theta_y, tx, tz);
                // CodeRotateByX(ty, tz, -1 * theta_x, ty, tz);

                // // 清空投影点向量
                // projectedPoints.clear();
                // // 将三维点投影到二维图像上
                // projectPoints(objPM, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
                // // 在图像上绘制投影点
                // for (unsigned int i = 0; i < projectedPoints.size(); ++i) 
                // {
                //     if (projectedPoints[i].x > 0 && projectedPoints[i].x < 640 && projectedPoints[i].y > 0 && projectedPoints[i].y < 480)
                //         circle(frame, projectedPoints[i], 3, Scalar(255, 0, 0), -1, 8);
                // }

                // // 创建字符数组，用于存储欧拉角和平移向量的字符串表示
                // char theta_z_name[20];
                // char theta_y_name[20];
                // char theta_x_name[20];
                // char x_name[20];
                // char y_name[20];
                // char z_name[20];
                // // 将欧拉角和平移向量转换为字符串
                // sprintf(z_name, "z:%d", int(tz * -1));
                // sprintf(y_name, "y:%d", int(ty * -1));
                // sprintf(x_name, "x:%d", int(tx * -1));
                // sprintf(theta_z_name, "theta_z_name%d", int(theta_z));
                // sprintf(theta_y_name, "theta_z_name%d", int(theta_y));
                // sprintf(theta_x_name, "theta_z_name%d", int(theta_x));
                // // 在图像上绘制欧拉角和平移向量的字符串
                // putText(frame, z_name, Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
                // putText(frame, y_name, Point(50, 150), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
                // putText(frame, x_name, Point(50, 200), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));


            } else {
                cv::putText(frame, "Local Track Failed", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0x27, 0xC1, 0x36), 2);
                light_track_flag = 0;
            }

        }
    }

    // 绘制虚线框的起始和结束点
    // cv::Point topLeft(400, 200);
    // cv::Point bottomRight(500, 250);

    // RCLCPP_INFO(this->get_logger(), "centerXY: %d %d %d %d", topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);

    // 绘制红色虚线框
    // for (int i = topLeft.x; i < bottomRight.x; i += 5) {
    //     cv::line(frame, cv::Point(i, topLeft.y), cv::Point(i + 2, topLeft.y), cv::Scalar(0, 0, 255), 1);
    //     cv::line(frame, cv::Point(i, bottomRight.y), cv::Point(i + 2, bottomRight.y), cv::Scalar(0, 0, 255), 1);
    // }
    // for (int i = topLeft.y; i < bottomRight.y; i += 5) {
    //     cv::line(frame, cv::Point(topLeft.x, i), cv::Point(topLeft.x, i + 2), cv::Scalar(0, 0, 255), 1);
    //     cv::line(frame, cv::Point(bottomRight.x, i), cv::Point(bottomRight.x, i + 2), cv::Scalar(0, 0, 255), 1);
    // }

    // 显示图像
    cv::imshow("Camera Image", frame);
    cv::waitKey(1);


    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(img_device));
    CUDA_CHECK(cudaFreeHost(img_host));
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    context->destroy();
    engine->destroy();
    runtime->destroy();
}


void UavTopicSubscrib::global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Global Position");
    RCLCPP_INFO(this->get_logger(), "Global Position: [Lat: %f, Lon: %f, Alt: %f]",
                msg->lat, msg->lon, msg->alt);
}




void UavTopicSubscrib::cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect)
{
    rect.x = max(0, pos.x - int(sz.x / 2));
    rect.y = max(0, pos.y - int(sz.y / 2));
    rect.width = int(sz.x);
    rect.height = int(sz.y);
}




void UavTopicSubscrib::getTarget2dPoinstion(const cv::RotatedRect & rect, vector<Point2f> & target2d, const cv::Point2f & offset) 
{
    Point2f vertices[4];
    rect.points(vertices);//把矩形的四个点复制给四维点向量
    Point2f lu, ld, ru, rd;
    sort(vertices, vertices + 4, [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });//从4个点的第一个到最后一个进行排序
    if (vertices[0].y < vertices[1].y) {
        lu = vertices[0];
        ld = vertices[1];
    }
    else {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
        ru = vertices[2];
        rd = vertices[3];
    }
    else {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}


//将空间点绕Z轴旋转
//输入参数 x y为空间点原始x y坐标
//thetaz为空间点绕Z轴旋转多少度，角度制范围在-180到180
//outx outy为旋转后的结果坐标
void UavTopicSubscrib::CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * 3.14 / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}
void UavTopicSubscrib::CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * 3.14 / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}
//将空间点绕X轴旋转
//输入参数 y z为空间点原始y z坐标
//thetax为空间点绕X轴旋转多少度，角度制，范围在-180到180
//outy outz为旋转后的结果坐标
void UavTopicSubscrib::CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * 3.14 / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}