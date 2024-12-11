# Autonomous-Intercept-Drone
An Autonomous Intercept Drone with Image-based Visual Servo.
wangzhaowu liaozipeng lituo
linjialin



## 1、启动仿真环境
### 1.1 编译PX4命令获取两架iris模型无人机和深度相机无人机
` ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "iris_depth_camera:1, iris:1"`
### 1.2 UDP通信
` MicroXRCEAgent udp4 -p 8888`


### 2、单机无人机
` make px4_sitl gazebo-classic`
