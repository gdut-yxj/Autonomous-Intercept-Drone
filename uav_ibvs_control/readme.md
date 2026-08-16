
# Precise Interception Multicopter (IBVS + PNG)

该部分为无人机视觉伺服拦截部分详解，具体内容参考 IEEE T-IE 2025 论文："Precise Interception Flight Targets by Image-based Visual Servoing of Multicopter"。  
该部分主要实现了论文中提出的结合 图像视觉伺服 (IBVS) 与 比例导引 (PNG) 的控制方案，通过单目相机实现对非合作飞行目标的拦截效果。

## 🚀 核心特性

- **IBVS + PNG 融合控制**：将比例导引律引入视觉伺服框架，通过控制速度向量角（Velocity Angle）而非直接位置控制，使拦截轨迹更平滑，显著降低末端过载。
- **视野保持控制器 (FOV Holding)**：专门设计了针对图像平面误差的补偿控制器，确保目标在拦截过程中始终处于相机视野中心。
- **多状态机管理**：包含起飞 (Take-off)、目标检测 (Detecting)、追踪拦截 (Tracking) 和目标丢失处理 (Loss Handling) 完整逻辑。
- **PX4 Offboard 模式集成**：完美兼容 PX4 固件，支持速度环 e 姿态角速率环切换控制。

## 🧠 算法原理

项目核心逻辑位于 `los_control.cpp` 中，主要分为三个协同工作的层面：

### 引导层 (Guidance Layer)

根据当前视线角 $q$（LOS angle）和速度角 $\sigma$，利用比例导引律（PNG）计算期望的速度方向。其核心原理是使速度矢量的转率与视线转率成正比，从而使视线角速率趋于零：

$$
\frac{d\sigma}{dt} = K \frac{dq}{dt}
$$

在代码实现的离散时间步中，期望的飞行速度角 $\sigma_d$ 计算公式为：

$$
\begin{cases} 
\sigma_{yd} = K_y(q_{y(k)} - q_{y(k-1)}) + \sigma_{y(k-1)} \\
\sigma_{zd} = K_z(q_{z(k)} - q_{z(k-1)}) + \sigma_{z(k-1)}
\end{cases}
$$

其中 $K_y, K_z$ 为导航增益（代码中对应 kv, kz），通过这种方式可以使无人机平滑地切入目标的拦截路径。

### 视野保持层 (FOV Holding Layer)

为提高拦截过程中的鲁棒性，系统通过控制无人机姿态和速度来最小化图像平面误差 $e = [e_x, e_y]^T$：

- **水平方向 ($e_x$) 控制**：设计偏航角速度 PD 控制器，使目标始终处于图像横向中心：

$$
w_{\psi} = k_p e_x + k_d \dot{e}_x
$$

- **纵向方向 ($e_y$) 控制**：通过调节前向期望速度 $v_d$ 来间接减少俯仰角剧烈变化带来的纵向误差波动：

$$
v_d = v_{now} + k_a
$$

最终实现控制目标：

$$
\begin{cases} 
e_x \rightarrow 0 \\
\Delta e_y \le \epsilon
\end{cases}
$$

确保拦截全过程的 2D 可见性。

### 控制层 (Control Layer)

- **速度模式**：用于初始阶段接近目标。
- **角速率/升力模式**：用于末端高动态拦截。系统计算期望加速度 $a_d$，并将其转化为期望姿态 $R_d$ 和期望升力 $f_d$：

$$
n_{fd} = \frac{a_d - g}{\|a_d - g\|}
$$

## 🎮 使用说明

### 1. 自动化流程

代码内部实现了一个全自动状态机（State），运行流程如下：

- **Take-off**: 节点启动后会自动尝试解锁（Arm）并切换至 Offboard 模式，升至预设的 `standby_height`。
- **Image Detect**: 此时程序会监听 `/fmu/out/hover_thrust_estimate`。注意： 必须等到 PX4 估计出稳定的悬停推力（hover_flag 为 true）后，系统才会允许进入追踪状态，以保证末端升力控制的准确性。
- **Tracking**: 当目标识别稳定超过 100 帧且目标未丢失时，进入 PNG 引导拦截。
- **Detect Loss**: 如果目标丢失超过 60 帧，无人机将自动进入悬停（Hover）状态以确保安全。

### 2. 关键参数配置

你可以通过修改 `los_control.hpp` 中的以下参数来调整表现：

- **foc**: 相机焦距（默认 544.0）。
- **kv / kz**: PNG 导航增益（默认 7.0）。
- **k1 / k2**: 视野保持器的 PD 参数。
- **d_gain**: 拦截时的速度增益（默认 4.0 m/s）。

### 3. 话题接口

| 类型   | 话题名称                         | 消息类型               | 说明                                                 |
|--------|----------------------------------|------------------------|------------------------------------------------------|
| 订阅   | /camera_detect_result            | uav_common_msg::RectMsg | 目标检测框坐标（x, y, w, h）。备注： 该话题需由外部 AI 识别程序（如 YOLO）处理图像后通过 ROS 2 实时输出。 |
| 订阅   | /fmu/out/hover_thrust_estimate   | HoverThrustEstimate     | 获取 PX4 估计的悬停推力值                              |
| 发布   | /fmu/in/vehicle_rates_setpoint   | VehicleRatesSetpoint    | 发送末端姿态角速率及升力指令                           |
| 发布   | /los_data                        | uav_common_msg::Data    | 算法内部变量监控（视线角、速度角等）                   |

## 📁 仓库结构

```
.
├── include/
│   └── los_control.hpp      # 节点类定义、状态机枚举及参数配置
├── src/
│   └── los_control.cpp      # 核心逻辑实现（PNG引导算法、离散化控制逻辑）
└── CMakeLists.txt           # 编译配置
```

## 🛠️ 环境要求

- **操作系统**: Ubuntu 20.04/22.04
- **ROS 2**: Foxy / Humble
- **固件**: PX4 Autopilot
- **依赖库**:
  - Eigen3 (矩阵运算)
  - GeographicLib
  - px4_msgs & uav_common_msg (自定义检测消息)

## 🔧运行节点  
确保你的检测算法（如 YOLO）正在发布 `/camera_detect_result` 话题，且微型飞行器（MAV）已连接：

```bash
ros2 run uav_ibvs_control uav_ibvs_control
```



