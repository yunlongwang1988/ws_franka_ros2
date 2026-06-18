# Franka FR3 遥操作控制工作空间 (ws_franka_ros2)

## 概述

基于 franka_ros2 的 Franka FR3 机械臂遥操作控制工作空间，实现 Sigma7 力反馈手柄对 FR3 的增量式笛卡尔位姿遥操作。

核心功能：
- 增量式笛卡尔位姿遥操作（零跳变启动）
- 二阶临界阻尼滤波保证运动平滑
- 各轴独立缩放与工作空间边界保护
- 姿态缩放跟随（防奇异点）
- 夹爪遥操作（独立节点，不干扰 RT 循环）

## 目录结构

```
ws_franka_ros2/
├── src/
│   ├── franka_bringup/          # 启动文件与配置
│   ├── franka_description/      # URDF 模型
│   ├── franka_example_controllers/  # 遥操作控制器（核心）
│   ├── franka_fr3_moveit_config/    # MoveIt 运动规划配置
│   ├── franka_gazebo/           # Gazebo 仿真
│   ├── franka_gripper/          # 夹爪 action server
│   ├── franka_hardware/         # 硬件接口（libfranka）
│   ├── franka_msgs/             # 消息/服务/动作定义
│   ├── franka_robot_state_broadcaster/  # 机器人状态广播
│   └── franka_semantic_components/      # 语义组件接口
├── gripper_bridge.py            # Sigma7 夹爪 → Franka 夹爪桥接
├── launch_gripper.py            # 夹爪一键启动脚本
└── .gitignore
```

## 核心控制器：cartesian_pose_example_controller

位于 `src/franka_example_controllers/`，实现增量式遥操作。

### 工作原理

1. 订阅 `/sigma1/pose`（Sigma7 末端位姿，~500Hz）
2. 首帧自动锚定 sigma 和 franka 位姿（零跳变）
3. 计算 sigma 相对锚点的增量，乘以各轴缩放因子
4. 叠加到 franka 锚点位姿，做工作空间边界保护
5. 通过二阶临界阻尼滤波平滑输出到硬件

### 参数配置

| 参数 | 当前值 | 说明 |
|------|--------|------|
| scale_x | -5.0 | X轴缩放（负=方向取反）|
| scale_y | -2.5 | Y轴缩放 |
| scale_z | 6.5 | Z轴缩放 |
| scale_ori | 0.6 | 姿态缩放 |
| max_vel | 0.20 m/s | 最大线速度 |
| max_ang_vel | 0.5 rad/s | 最大角速度 |
| wn_pos | 5.0 | 位置滤波自然频率 |
| wn_ori | 5.0 | 姿态滤波自然频率 |

### 工作空间边界（Franka base frame）

| 轴 | 最小值 (m) | 最大值 (m) |
|----|-----------|-----------|
| X | 0.20 | 0.75 |
| Y | -0.40 | 0.40 |
| Z | -0.22 | 0.70 |

## 夹爪控制

夹爪使用独立进程控制，避免干扰 arm 的 1kHz RT 循环。

- `gripper_bridge.py` — 订阅 `/sigma1/gripper_angle`，通过 action client 控制 franka gripper
- `launch_gripper.py` — 一键启动 franka_gripper_node + gripper_bridge

夹爪映射：
- Sigma7 gripper angle: [0, π/6] rad（0=闭合, 0.524=全开）
- 闭合阈值: < 0.05 rad → grasp（force=50N）
- 张开阈值: > 0.12 rad → open（width=0.08m）

## 编译

```bash
cd ~/ws_yunlong/ws_franka_ros2
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 启动流程

### 终端 1：启动 Franka 机械臂

```bash
cd ~/ws_yunlong/ws_franka_ros2
source install/setup.bash
ros2 launch franka_bringup example.launch.py controller_name:=cartesian_pose_example_controller
```

### 终端 2：启动 Sigma7（在 ws_franka_omega_real）

```bash
cd ~/ws_yunlong/ws_franka_omega_real
source install/setup.bash
ros2 run sigma7 sigma_main
```

### 终端 3：启动夹爪控制

方式 A — sigma_gripper_teleop（libfranka 直连，推荐）:
```bash
cd ~/ws_yunlong/ws_franka_omega_real
source install/setup.bash
ros2 run sigma_gripper_teleop sigma_gripper_teleop_node --ros-args -p robot_ip:=172.16.0.2
```

方式 B — franka_gripper action server + Python 桥接:
```bash
cd ~/ws_yunlong/ws_franka_ros2
source install/setup.bash
ros2 launch launch_gripper.py
```

## 安全机制

1. **增量式遥操作** — 启动零跳变，sigma 首帧自动锚定
2. **二阶临界阻尼滤波** — 位置/姿态独立滤波，jerk/acc/vel 三级限制
3. **工作空间边界** — 超出范围自动钳位
4. **姿态缩放** — scale_ori=0.6 防止接近奇异点
5. **libfranka 内置** — rate limiter + low-pass filter 启用
6. **夹爪隔离** — 独立进程，不影响 RT 控制循环

## 硬件配置

- **机器人**: Franka FR3, IP: 172.16.0.2
- **操作系统**: Ubuntu 22.04 + PREEMPT_RT 内核
- **ROS**: ROS2 Humble
- **libfranka**: 0.18.x
- **控制频率**: 1kHz (RT priority 98)

## 关键修改说明

相比上游 franka_ros2，本仓库主要修改了：

1. **franka_example_controllers** — 完全重写 `cartesian_pose_example_controller` 为增量式遥操作
2. **franka_hardware/robot.hpp** — 启用 `cartesian_pose_command_rate_limit_active_` 和 `cartesian_pose_low_pass_filter_active_`
3. **franka_bringup/config** — 调整控制器配置和 RT 参数

