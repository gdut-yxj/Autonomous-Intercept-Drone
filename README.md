# Autonomous Intercept Drone
### Image-based Visual Servo Autonomous Intercept Drone System (Autonomous Intercept Drone with Image-based Visual Servo)

<div align="center">

**English** | [**中文**](README.zh-CN.md)

[![Bilibili](https://img.shields.io/badge/Bilibili-观看视频演示-ff69b4?style=for-the-badge&logo=bilibili)](https://www.bilibili.com/video/BV1M8QVYHE39/?spm_id_from=333.1387.homepage.video_card.click)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-blue.svg)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FFoxy-green.svg)](https://docs.ros.org/en/humble/)
[![TensorRT](https://img.shields.io/badge/TensorRT-Accelerated-76B900.svg)](https://developer.nvidia.com/tensorrt)
[![YOLO11](https://img.shields.io/badge/YOLO-v11-yellow.svg)](https://github.com/ultralytics/ultralytics)

</div>

> This project is an experimental verification platform for autonomous drone interception tasks. It integrates visual perception, target detection, proportional navigation guidance (PNG) algorithms, and flight control methods. The project aims to provide a complete simulation reference solution for autonomous drone interception, guidance algorithm research, and visual servo control, and is suitable for learning and academic research.
---

## Project Overview

Developed in the ROS 2 environment and combined with the PX4 firmware and the Gazebo simulator, this system implements a closed loop from target discovery to simulated interception. The main features include:

*   **Visual Target Detection**: Uses a trained YOLO model and a tracking model (LightTrack) to recognize and track drone targets in the field of view in real time.
*   **Proportional Navigation Guidance**: Generates intercept trajectories based on the Proportional Navigation Guidance (PNG) law to achieve precise lead interception of dynamic targets.
*   **Simulation Verification System**: Built on PX4 software-in-the-loop (SITL), it supports offline algorithm verification and parameter tuning in the Gazebo environment.
*   **Data Analysis Tools**: Built-in kinematics and visual metrics recording scripts are provided for quantitative evaluation of guidance law accuracy and visual tracking stability.

---

## Simulation Demonstration

### 1. Full Dynamic Interception Process
<div align="center">
  <img src="assets/output.gif" alt="Interception dynamic demonstration" width="80%">
  <p><i>(The PNG algorithm predicts the target trajectory to achieve precise physical collision interception.)</i></p>
  <p><b>📺 <a href="https://www.bilibili.com/video/BV1M8QVYHE39/">Click here to watch the full HD demonstration video on Bilibili</a></b></p>
</div>

### 2. Small-Target Drone Detection Framework
<div align="center">
  <img src="assets/小目标无人机检测框架.png" alt="Small-target detection framework" width="80%">
  <p><i>(Vision processing pipeline optimized for drone "point targets".)</i></p>
</div>

---

## System Architecture and Logic

The system adopts a layered ROS 2 architecture, ensuring high flexibility and real-time performance:

<div align="center">
  <img src="assets/框架.png" alt="Architecture diagram" width="80%">
</div>

| Module | Description | Key Packages | Status/Notes |
| :--- | :--- | :--- | :--- |
| **Perception Layer** | Target detection, pixel error computation, line-of-sight angle extraction | `uav_vision_dectect`, `uav_vision_png` | **Current primary solution** |
| **Guidance Layer** | PNG guidance law computation, lead compensation, trajectory generation | `uav_png_intercept` | **Pure PNG solution (does not rely on vision)** |
| **Learned Guidance** | Reinforcement learning based GRU policy guidance, replacing traditional PNG | `uav_rl_guidance` | 🆕 **RL solution (replaces vision_png)** |
| **Control Layer** | PX4 Offboard interface, velocity/attitude closed-loop control | `uav_vehicle_controller`, `px4_ros_com` | Core control foundation |
| **Simulation Layer** | Target drone dynamics simulation, multiple motion mode support, intercept environment generation | `uav_target_sim` | Simulation support |
| **Earlier Solution** | Image-based visual servo control solution | `uav_ibvs_control` | **Deprecated (code kept for reference only)** |

---

## Quantitative Interception Performance Analysis

The following shows the system performance in typical interception tasks, which can serve as a reference baseline for algorithm optimization:

### Kinematic Performance
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/1_3D_Trajectory.png" width="300px"><br><b>3D Interception Trajectory</b></td>
      <td><img src="assets/plots_output/2_Relative_Distance.png" width="300px"><br><b>Relative Distance Convergence</b></td>
      <td><img src="assets/plots_output/3_Velocity.png" width="300px"><br><b>Interceptor Velocity Vector</b></td>
    </tr>
    <tr>
      <td>Shows the lead interception in 3D space</td>
      <td>Verifies that the intercept distance converges to &lt;0.2m</td>
      <td>Reflects the efficient use of thrust by the guidance law</td>
    </tr>
  </table>
</div>

### Visual Tracking Performance
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/4_LOS_PNG_Angles.png" width="300px"><br><b>Line-of-Sight (LOS) Angle Evolution</b></td>
      <td><img src="assets/plots_output/5_Pixel_Error.png" width="300px"><br><b>Visual Center Tracking Error</b></td>
      <td><img src="assets/plots_output/6_2D_Top_View.png" width="300px"><br><b>2D Top-Down Interception Path</b></td>
    </tr>
    <tr>
      <td>Convergence stability analysis of the guidance law</td>
      <td>Robustness of the detection algorithm during dynamic motion</td>
      <td>Typical lead compensation effect demonstration</td>
    </tr>
  </table>
</div>

---

## Quick Start

### 1. Environment Requirements
*   **ROS 2 Version**: Humble
*   **Gazebo**: Gazebo Harmonic (gz-sim 8.12.0)
*   **PX4 Firmware**: v1.16 (**Micro-XRCE-DDS-Agent** version v2.4.x recommended)
*   **Dependencies**: OpenCV, onnx, colcon

### 2. Build the Project
```bash
# Enter the workspace and build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Running the Interception Task (Step Sequence)
Please run each node in a separate terminal in the following order:


0. **Start the Simulation Environment**:
   Before starting, copy the simulation resources from this repository to the PX4-Autopilot directory:

   ```bash
   # Step 1: Copy the run_swarm.sh script from this repository to the PX4-Autopilot root directory (if not already copied)
   cp ros2_ws/src/run_swarm.sh ~/PX4-Autopilot/

   # Step 2: Copy the custom world file to the PX4 Gazebo worlds directory
   cp ros2_ws/src/assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   ```

   > The above steps only need to be performed once. After that, to start the simulation, simply enter the PX4-Autopilot directory and run the startup script:
   ```bash
   cd ~/PX4-Autopilot
   ./run_swarm.sh
   ```

   This script automatically starts the Gazebo simulation server, two drones (px4_1 interceptor + px4_2 target), the MicroXRCEAgent, and the camera topic bridge.


1. **Start the Target Simulator**:
   ```bash
   # Default circular motion
   ros2 run uav_target_sim uav_target_sim

   # Or select the motion mode and movement range via parameters
   ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=sinusoidal -p max_range:=10.0

   # You can also use the launch file
   ros2 launch uav_target_sim target_sim.launch.py motion_mode:=random_walk
   ```

   **Supported target motion modes**:

   | Parameter | Motion Mode | Kinematic Formula | Characteristics |
   | :--- | :--- | :--- | :--- |
   | `circle` | Uniform circular | R=5m, w=0.5rad/s | Default mode, uniform circular motion with a radius of 5m |
   | `sinusoidal` | Sinusoidal maneuver | a=0.5*sin(0.5t) m/s<sup>2</sup> | Forward uniform 1m/s + lateral sinusoidal acceleration, periodic evasion |
   | `random_walk` | Random walk | v(t+dt) = v(t) + N(0, 0.2) | Random velocity perturbation, speed limited to 3m/s, irregular escape |

   > The `max_range` parameter (default 10m) controls the target movement range to prevent the target from flying out of the interceptor's field of view.

2. **Start Vision-Guided Interception**:
   ```bash
   # Recommended to run directly (uses the default parameters in the .hpp file)
   ros2 run uav_vision_png uav_vision_png

   # Or use the launch file to load parameters from config/params.yaml
   ros2 launch uav_vision_png vision_png.launch.py
   ```

   > **Parameter Configuration Notes**: The parameter priority of the `uav_vision_png` node is:
   >   **launch CLI > params.yaml > .hpp member initial values**
   >
   > - When started with `ros2 run`: parameters take the default values in `.hpp`; **`params.yaml` is not loaded automatically**, so you need to pass parameters manually via `--ros-args -p param_name:=value`
   > - When started with `ros2 launch`: the launch file automatically loads `config/params.yaml`, and the YAML values override the `.hpp` defaults
   > - During debugging, use `ros2 param get /uav_vision_png <param_name>` to inspect the currently effective value
   > - See `uav_vision_png/config/params.yaml` for tunable parameters, including PNG gain, FOV compensation, takeoff altitude, etc.

2-Alt. **Start RL Learned Guidance Interception (Alternative)** 🆕:
   `uav_rl_guidance` is a GRU policy guidance node trained with reinforcement learning (BC + PPO), used to replace the traditional PNG algorithm.
   It maintains the same topic interfaces, state machine, and CSV statistics format as `uav_vision_png`, so it can be used as a seamless replacement.
   It uses ONNX Runtime for inference, with zero PyTorch dependencies and a pure C++ implementation.

   ```bash
   # Method 1: Launch via the launch file (automatically loads config/params.yaml)
   ros2 launch uav_rl_guidance rl_guidance.launch.py

   # Method 2: Run directly (uses the default parameters in the code)
   ros2 run uav_rl_guidance uav_rl_guidance

   # A/B baseline mode: use the built-in PNG throughout, without loading the policy model (equivalent to uav_vision_png)
   ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

   # Bench debugging: skip takeoff and enter SEARCHING directly
   ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
   ```

   > **Architecture Notes**: The RL policy outputs velocity commands during the INTERCEPT phase; if the policy behaves abnormally (watchdog), it automatically falls back to the built-in PNG controller.
   > The policy model (ONNX `.onnx`) is stored under `uav_rl_guidance/models/`.
   > The training code is available at [AeroIntercept](https://github.com/Eaglewzw/AeroIntercept). To update the model, run `python3 uav_rl_guidance/src/export_onnx.py`.

3. **Start Vision Detection**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```
---

---

> **Project Maintainer's Note**: If you need more details about the algorithm derivation (PNG), please refer to the `include/` header files in each package or contact the development team.
