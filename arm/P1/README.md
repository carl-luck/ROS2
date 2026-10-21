# P1：基于 UR5e 仿真的轨迹执行与测试

基于 Universal Robots 官方 ROS2 Gazebo 仿真工程，使用 Python Action 客户端执行关节轨迹，记录控制器反馈，分析跟踪误差，并验证主动取消及客户端等待超时后的取消流程。

本项目仅用于仿真，不适用于直接控制实机。

## 官方组件与本人实现

官方工程提供：

- UR5e 模型、仿真启动及控制接口。
- 现成轨迹控制器及 FollowJointTrajectory Action 服务端。

本人增加：

- 固定关节位置往返客户端与运动时间参数。
- 按关节名称读取实际位置、检查模型位置限位。
- 控制器期望、实际位置和误差的 CSV 记录。
- 执行状态、耗时和终点检查的 JSON 记录。
- 六关节误差分析及底座轨迹可视化。
- 主动取消和客户端等待超时后自动取消实验。

本项目使用现成控制器，未自行实现机械臂底层控制算法。

## 已验证环境

- Ubuntu 22.04，运行于 VMware 虚拟机。
- ROS2 Humble。
- Python 3.10.12。
- Gazebo Sim 6 核心库：libignition-gazebo6，6.18.0-1~jammy。
- 机器人：UR5e。
- 官方源码提交：
  e49336eb369a3e75fd31753512d4afb3c0c1eb6f

官方源码：
https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

源码来源与提交记录在 dependencies.repos 中。
该文件不代表已锁定全部系统和 Python 依赖版本。

## 目录

```text
P1/
├── README.md
├── dependencies.repos
├── scripts/
│   ├── move_joint.py
│   ├── round_trip.py
│   ├── analyze_round_trip.py
│   ├── plot_round_trip.py
│   └── cancel_motion.py
├── data/
├── figures/
└── ros2_ws/
```

官方源码位于 ros2_ws/src/ur_simulation_gz，由外层仓库忽略。
构建产物 build、install、log 不纳入作品集。
scripts 中的 before_* 等文件为历史备份，不是运行入口。

## 控制接口

Action：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

类型：

```text
control_msgs/action/FollowJointTrajectory
```

实际使用的控制器：

- joint_state_broadcaster
- joint_trajectory_controller

关节状态来自 /joint_states。

## 在现有已构建环境中运行

以下命令在 Ubuntu 中执行。
图形仿真从 Ubuntu 桌面终端启动。

### 启动仿真

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```

保持该终端运行。另开终端操作客户端。
确保 Gazebo 未暂停，不同时运行多个运动客户端。

### 检查状态

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash

ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
```

确认控制器 active，并读取实际位置。
不假定重启后的姿态与上次结束时相同。

### 固定位置往返

```bash
python3 ~/ROS2/arm/P1/scripts/round_trip.py --duration 5
```

或在上一条完全结束后运行：

```bash
python3 ~/ROS2/arm/P1/scripts/round_trip.py --duration 9
```

底座目标顺序为：

```text
当前位置 → A：0.15 rad → B：0.30 rad → A：0.15 rad
```

其他关节以本次实验开始时的位置为目标。

duration 表示单段总仿真时间：
第一个轨迹点在第 1 秒，终点在第 5 秒或第 9 秒。
因此计划移动时间分别约为 4 秒、8 秒。

重复运行使用固定目标，不累计增加角度。
若开始时已在 A，第一段可能基本不动。

程序在 data 中生成原始反馈 CSV 和同名前缀的 _run.json。

### 分析数据

```bash
python3 ~/ROS2/arm/P1/scripts/analyze_round_trip.py \
  ~/ROS2/arm/P1/data/round_trip_T9s_20261021_123838_521138.csv
```

分析脚本要求原始 CSV 旁存在匹配的 _run.json。
对六个关节分别计算全段和计划运动阶段指标，输出 _metrics.csv。

### 绘图

```bash
python3 ~/ROS2/arm/P1/scripts/plot_round_trip.py \
  ~/ROS2/arm/P1/data/round_trip_20261018_035041_055973.csv
```

绘图需要 matplotlib，生成 PNG 和 SVG。
当前绘图脚本展示底座关节，横轴为相对首帧反馈的仿真时间。

### 主动取消

```bash
python3 ~/ROS2/arm/P1/scripts/cancel_motion.py --mode active
```

在目标接受后约 3 秒仿真时间请求取消。

### 客户端等待超时后取消

```bash
python3 ~/ROS2/arm/P1/scripts/cancel_motion.py --mode timeout
```

客户端等待结果的墙钟时间上限为 5 秒。
若等待超时，自动请求取消并检查最终状态及取消后的关节运动。

取消实验目标为底座 0.30 rad，从实际位置开始。
当前位置过于接近目标或变化超出实验范围时，脚本拒绝发送。
取消后不自动返回起点。

这些普通 Python 脚本保存后可直接重跑，无需重新 colcon 构建。

## 指标口径

跟踪误差比较同一条控制器反馈中的期望与实际位置。
已检查数据中 controller_error 等于 desired - actual。

分析指标：

- 样本 RMSE：按记录样本平均。
- 时间加权 RMSE：平方误差按仿真时间梯形积分后除以覆盖时长，再开方。
- 最大绝对误差：记录样本中的最大绝对误差。
- IAE：绝对误差按仿真时间梯形积分，单位 rad·s。

全段与计划运动阶段分开统计。
阶段划分使用控制器 desired.time_from_start。
积分只覆盖已记录样本范围，不外推未采样端点。

执行耗时来自客户端观测，包含通信和处理过程。
墙钟时间与仿真时间分开记录。
Gazebo 暂停或运行慢可能造成等待超时，不能直接判为控制故障。

## 已验证结果

### 运动时间对比

以下为两种时间条件各一次的底座运动阶段结果。
比较 B 和 A_return；A_start 的起点可能不同，不用于公平对比。

| 条件 | 计划移动时间 | 时间加权 RMSE | 最大绝对误差 | IAE |
|---|---:|---:|---:|---:|
| 单段 5 秒 | 约 4 秒 | 约 0.000824 rad | 约 0.001125 rad | 约 0.003 rad·s |
| 单段 9 秒 | 约 8 秒 | 约 0.000412 rad | 约 0.0005625 rad | 约 0.003 rad·s |

对应数据：

- round_trip_T5s_20261020_111953_093511.csv
- round_trip_T9s_20261021_123838_521138.csv

在本次条件下，移动时间加倍后，RMSE 和最大误差约减半，
但累计绝对误差 IAE 基本不变。

各段采样覆盖边界略有不同。
尚未完成系统重复测试，不能据此宣称重复性或可靠性已验证。
其他五个关节主要保持姿态，其数据不能证明运动跟踪性能。

### 取消实验

| 实验 | 取消接受 | 最终状态 | 观察最大速度 | 最大位置变化范围 |
|---|---|---|---:|---:|
| 主动取消 | True | CANCELED，5 | 0.00058171 rad/s | 0.00001047 rad |
| 等待超时后取消 | True | CANCELED，5 | 0.00046578 rad/s | 0.00000838 rad |

对应记录：

- cancel_motion_20261021_212445_268456.json
- cancel_motion_20261021_214837_294079.json

两次均观察了约 1 秒仿真时间，并满足本实验设定的：

- 最大绝对速度不超过 0.001 rad/s。
- 最大关节位置变化范围不超过 0.001 rad。

这些是实验检查阈值，不是厂家性能标准。
取消不代表瞬间静止，也不是实机安全急停。

data 中保留了调试阶段的失败记录，不计入通过案例。

## 当前限制

- 未进行系统重复测试。
- 尚无一键批量执行与汇总流程。
- 尚未完成从全新环境安装开始的独立复现验证。
- 未实现 MoveIt 规划、碰撞检查、避障或抓取。
- 未接入 MPC、ADRC，也未自行开发底层控制器。
- 未进行实机验证或末端定位精度验证。