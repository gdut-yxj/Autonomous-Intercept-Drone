//
// Created by verse on 24-10-9.
//
#include "uav_topic_subscrib.hpp"

#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <chrono>
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



    uav_detect_result_publisher_ = this->create_publisher<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10);

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
        if (best_score > 0.3) {
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

        }

    }


    if(light_track_flag == 1){

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
            } else {
                cv::putText(frame, "Local Track Failed", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0x27, 0xC1, 0x36), 2);
                light_track_flag = 0;
            }

        }
    }

    int width = frame.cols;
    int height = frame.rows;

    int centerX = width / 2;
    int centerY = height / 2;


    // RCLCPP_INFO(this->get_logger(), "centerXY: %d , %d", centerX, centerY);

    // 绘制虚线框的起始和结束点
    int boxSize = 100;
    cv::Point topLeft(centerX - boxSize / 2, centerY - boxSize / 2);
    cv::Point bottomRight(centerX + boxSize / 2, centerY + boxSize / 2);

    RCLCPP_INFO(this->get_logger(), "centerXY: %d %d %d %d", topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);

    // 绘制红色虚线框
    for (int i = topLeft.x; i < bottomRight.x; i += 5) {
        cv::line(frame, cv::Point(i, topLeft.y), cv::Point(i + 2, topLeft.y), cv::Scalar(0, 0, 255), 1);
        cv::line(frame, cv::Point(i, bottomRight.y), cv::Point(i + 2, bottomRight.y), cv::Scalar(0, 0, 255), 1);
    }
    for (int i = topLeft.y; i < bottomRight.y; i += 5) {
        cv::line(frame, cv::Point(topLeft.x, i), cv::Point(topLeft.x, i + 2), cv::Scalar(0, 0, 255), 1);
        cv::line(frame, cv::Point(bottomRight.x, i), cv::Point(bottomRight.x, i + 2), cv::Scalar(0, 0, 255), 1);
    }


    uav_common_msg::msg::RectMsg pub_uav_result_rect;
    pub_uav_result_rect.header = std_msgs::msg::Header();
    pub_uav_result_rect.x = uav_result_rect.x;
    pub_uav_result_rect.y = uav_result_rect.y;
    pub_uav_result_rect.width = uav_result_rect.width;
    pub_uav_result_rect.height = uav_result_rect.height;
    uav_detect_result_publisher_->publish(pub_uav_result_rect);

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
