# Franka FR3 遥操作控制工作空间 (ws_franka_ros2)

## 概述

基于 franka_ros2 的 Franka FR3 机械臂遥操作，实现 Sigma7 力反馈设备对 FR3 的增量式笛卡尔位姿控制。

主要功能：
- 增量式笛卡尔位姿遥操作；
- 二阶临界阻尼滤波保证运动平滑；
- 工作空间边界保护；
- 姿态遥操作防止奇异点；
- 夹爪单独遥操作；

## 目录结构

```
ws_franka_ros2/
├── src/
│   ├── franka_bringup/                  # 启动文件与配置
│   ├── franka_description/              # URDF 模型
│   ├── franka_example_controllers/      # 遥操作控制器
│   ├── franka_fr3_moveit_config/        # MoveIt 运动规划配置
│   ├── franka_gazebo/                   # Gazebo 仿真
│   ├── franka_gripper/                  # 夹爪 action server
│   ├── franka_hardware/                 # 硬件接口（libfranka）
│   ├── franka_msgs/                     # 消息/服务/动作定义
│   ├── franka_robot_state_broadcaster/  # 机器人状态 broadcaster
│   └── franka_semantic_components/      # 语义组件接口
├── gripper_bridge.py                    # Sigma7 夹爪 → Franka 夹爪
├── launch_gripper.py                    # 夹爪启动脚本
└── .gitignore
```

## 控制器：cartesian_pose_example_controller

位于 `src/franka_example_controllers/`，实现增量式遥操作。

### 工作原理

1. 订阅 `/sigma1/pose`（Sigma7 末端位姿，~500Hz）；
2. 启动时获取 sigma 和 franka 位姿；
3. 计算 sigma 相对增量，乘以各轴缩放因子；
4. 叠加到 franka 当前位姿，并做好工作空间边界保护；
5. 通过二阶临界阻尼滤波平滑输出到执行器；

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

夹爪独立控制，避免干扰 franka arm 的 1kHz 循环。

- `gripper_bridge.py` — 订阅 `/sigma1/gripper_angle`，通过 action client 控制 franka gripper；
- `launch_gripper.py` — 启动 franka_gripper_node + gripper_bridge；

夹爪映射：
- Sigma7 gripper angle: [0, π/6] rad（0=闭合, 0.524=全开）；
- 闭合阈值: < 0.05 rad → grasp（force=50N）；
- 张开阈值: > 0.12 rad → open（width=0.08m）；

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

### 终端 2：启动 Sigma7（在 ws_franka_omega_real 目录下）

```bash
cd ~/ws_yunlong/ws_franka_omega_real
source install/setup.bash
ros2 run sigma7 sigma_main
```

### 终端 3：启动夹爪控制

方式 A — sigma_gripper_teleop（通过 libfranka 启动）:
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

1. **增量式遥操作** — 防止启动时位置变化过大；
2. **二阶临界阻尼滤波** — 位置/姿态独立滤波，jerk/acc/vel 都进行限制；
3. **工作边界** — 超出工作范围抱闸；
4. **姿态缩放** — scale_ori=0.6 防止接近奇异点；
5. **libfranka 底层** — rate limiter + low-pass filter；
6. **夹爪** — 独立控制；

## 硬件配置

- **机器人**: Franka FR3, IP: 172.16.0.2；
- **操作系统**: Ubuntu 22.04 + PREEMPT_RT 内核；
- **ROS**: ROS2 Humble；
- **libfranka**: 0.18.x 及其以上；
- **控制频率**: 1kHz (RT priority 98)；

## 修改说明

相比源franka_ros2，主要修改了：

1. **franka_example_controllers** — 重写 `cartesian_pose_example_controller` 为增量式遥操作；
2. **franka_hardware/robot.hpp** — 启用 `cartesian_pose_command_rate_limit_active_` 和 `cartesian_pose_low_pass_filter_active_`；
3. **franka_bringup/config** — 调整控制器配置；

