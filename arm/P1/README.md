# P1 基于 UR5e 仿真的轨迹执行与测试

基于 Universal Robots 官方 ROS2 Gazebo 仿真工程，编写 Python Action 客户端，实现固定关节轨迹执行、反馈记录、误差分析，以及主动取消和客户端等待超时后的取消处理。

Gazebo 用于物理仿真，RViz 用于模型与运动状态可视化。

本项目仅用于仿真，不适用于直接控制实机。

## 官方组件与本人实现

官方工程提供：

- UR5e 机器人模型与仿真启动配置。
- ros2_control 仿真接口及控制器配置。
- 轨迹控制器与 FollowJointTrajectory Action 服务端。

本人增加：

- 固定关节位置往返客户端及运动时间参数。
- 按关节名称读取实际位置、检查模型位置限位。
- 控制器期望位置、实际位置及误差的 CSV 记录。
- 运行配置、执行结果、耗时及终点检查的 JSON 记录。
- 六关节误差分析与底座轨迹可视化。
- 主动取消及客户端等待超时后自动取消实验。
- 批量离线分析脚本及分析成功、失败统计。

本项目使用现成控制器，未自行开发机械臂底层控制算法。

## 已验证环境

- Ubuntu 22.04，运行于 VMware 虚拟机。
- ROS2 Humble。
- Python 3.10.12。
- Gazebo Sim 6 核心库：libignition-gazebo6，6.18.0-1~jammy。
- RViz：用于机器人模型和运动状态显示。
- 机器人：UR5e。
- 开发方式：VS Code Remote-SSH。
- 数据分析与绘图：Python、Matplotlib。

官方源码：

https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

实际运行的源码提交：

```text
e49336eb369a3e75fd31753512d4afb3c0c1eb6f
```

来源和准确提交记录在 dependencies.repos 中。
该清单不代表已锁定全部系统和 Python 依赖版本。

## 目录

```text
P1/
├── README.md
├── dependencies.repos
├── scripts/
│   ├── move_joint.py
│   ├── round_trip.py
│   ├── analyze_round_trip.py
│   ├── batch_analyze.py
│   ├── plot_round_trip.py
│   └── cancel_motion.py
├── data/
├── figures/
└── ros2_ws/
```

- scripts：运动、分析、绘图与取消实验脚本。
- data：原始反馈、运行记录和分析结果。
- figures：PNG、SVG 结果图。
- ros2_ws：仿真工作空间。
- scripts 中名称包含 before 的文件为历史备份，不是运行入口。

官方源码位于 ros2_ws/src/ur_simulation_gz，由外层仓库忽略。
build、install、log 等构建产物不纳入作品集。

## 控制接口

Action 名称：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

Action 类型：

```text
control_msgs/action/FollowJointTrajectory
```

使用的控制器：

- joint_state_broadcaster
- joint_trajectory_controller

关节状态来自 /joint_states。

客户端负责发送任务、记录反馈、判断结果和请求取消。
轨迹控制器提供 Action 服务端并执行轨迹。

## 在现有已构建环境中运行

以下命令均在 Ubuntu 中执行。
图形仿真从 Ubuntu 桌面终端启动，客户端与分析操作主要在 VS Code 远程终端执行。

### 启动仿真

在 Ubuntu 桌面终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```

保持这个终端运行，另开终端执行后续操作。
确保 Gazebo 未暂停，不重复启动同一套仿真。

RViz 用于显示模型和运动，显示正常不等于控制或物理仿真必然正常。

### 检查控制器与实际状态

在另一个 Ubuntu 终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash

ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
```

确认两个控制器均为 active，并收到完整关节状态。
不假定重启后仍停在上次结束位置。

### 固定位置往返

```bash
python3 ~/ROS2/arm/P1/scripts/round_trip.py --duration 5
```

或在前一个脚本完全退出后执行：

```bash
python3 ~/ROS2/arm/P1/scripts/round_trip.py --duration 9
```

底座目标顺序：

```text
当前位置 → A：0.15 rad → B：0.30 rad → A：0.15 rad
```

其他五个关节以本次实验开始时的位置为目标。

duration 表示单段总仿真时间：

- 第一个轨迹点在第 1 秒，位置为本段开始时的实际位置。
- 终点在第 5 秒或第 9 秒。
- 计划移动时间分别约为 4 秒、8 秒。
- 两个轨迹点的速度均设置为零。

重复运行使用固定目标，不累计增加角度。
若开始时已在 A，第一段可能基本不动。
程序会检查目标限位和本实验允许的位置变化范围。

运行后在 data 中生成：

```text
round_trip_T时间_日期时间.csv
round_trip_T时间_日期时间_run.json
```

CSV 保存反馈过程，JSON 保存配置、Action 结果、耗时和终点检查。

### 分析一组数据

```bash
python3 ~/ROS2/arm/P1/scripts/analyze_round_trip.py \
  ~/ROS2/arm/P1/data/round_trip_T9s_20261021_123838_521138.csv
```

传入原始反馈 CSV，不是 _summary.csv 或 _metrics.csv。
同目录必须存在匹配的 _run.json。

脚本检查时间、六关节数据对齐和帧数，
对每段的六个关节分别统计全段和计划运动阶段指标，
生成同名前缀的 _metrics.csv。

完整三段实验输出：

```text
3 段 × 6 关节 × 2 种统计范围 = 36 行指标
```

只分析已有文件时，不需要启动仿真或加载 ROS2 环境。

### 批量分析数据

```bash
python3 ~/ROS2/arm/P1/scripts/batch_analyze.py
```

脚本自动查找 data 中的 round_trip_T*_run.json，
定位对应原始 CSV，并逐组调用 analyze_round_trip.py。

功能包括：

- 自动处理符合命名规则的多组数据。
- 缺少 CSV 或分析失败时，记录失败并继续下一组。
- 最后输出分析成功、失败数量及失败清单。
- 不修改原始 CSV 和运行 JSON。
- 重复执行会覆盖同名 _metrics.csv。

已使用两组配套 CSV 与 JSON 验证批量分析流程，分析成功 2 组、失败 0 组。

这里的“分析成功”表示分析程序完成，
不等于对应运动实验通过；执行结论仍需查看运行记录。

当前实现的是批量离线分析，
尚未实现批量运动执行或跨实验指标汇总。

### 绘制底座曲线

```bash
python3 ~/ROS2/arm/P1/scripts/plot_round_trip.py \
  ~/ROS2/arm/P1/data/round_trip_20261018_035041_055973.csv
```

绘图需要 Matplotlib，在 figures 中生成 PNG 和 SVG。

上排显示期望与实际位置，下排显示控制器跟踪误差。
横轴为相对各段首帧反馈的仿真时间，不是精确的轨迹起始时间。

### 主动取消

保持仿真运行，在已加载 ROS2 环境的终端执行：

```bash
python3 ~/ROS2/arm/P1/scripts/cancel_motion.py --mode active
```

目标被接受后约 3 秒仿真时间，请求取消。

### 客户端等待超时后取消

```bash
python3 ~/ROS2/arm/P1/scripts/cancel_motion.py --mode timeout
```

客户端等待结果的墙钟时间上限为 5 秒。
若等待超时，自动请求取消，再检查最终状态和关节运动。

这是人为缩短等待上限的异常处理实验，不代表控制器故障。

两种取消实验均：

- 从实际位置开始，底座目标为 0.30 rad。
- 当前位置过于接近目标或变化超出实验范围时，拒绝发送。
- 分别检查取消请求被接受、最终进入 CANCELED、取消后运动满足观察阈值。
- 保存 JSON 记录。
- 取消后不自动返回起点。

不要同时运行多个运动客户端。
直接运行的 Python 脚本保存后可重跑，无需重新 colcon 构建。
Ctrl+C 中断客户端不保证服务端任务已经取消。

## 指标定义

跟踪误差比较同一条控制器反馈中的期望与实际位置：

```text
error = desired - actual
```

已检查实验数据中的控制器误差与该差值一致。
不把运动中实际位置与最终目标的差称为轨迹跟踪误差。

统计指标：

| 指标 | 计算口径 | 单位 |
|---|---|---|
| 样本 RMSE | 对记录样本的平方误差平均后开方 | rad |
| 时间加权 RMSE | 平方误差按仿真时间梯形积分，除以覆盖时长后开方 | rad |
| 最大绝对误差 | 已记录样本中的最大绝对误差 | rad |
| IAE | 绝对误差按仿真时间梯形积分 | rad·s |

全段与计划运动阶段分别统计。
阶段划分使用反馈中的 desired.time_from_start。
积分仅覆盖所选样本的实际时间范围，不外推未采样端点。

执行耗时来自客户端观测，包含通信与处理过程。
仿真时间和墙钟时间分开记录。
Gazebo 暂停或运行慢可能导致等待超时，不能直接归为控制故障。

## 已验证结果

### 运动时间对比

两种时间条件各分析一次，比较相同转角的 B、A_return。
A_start 的起点可能不同，不用于公平对比。

| 条件 | 计划移动时间 | 时间加权 RMSE | 最大绝对误差 | IAE |
|---|---:|---:|---:|---:|
| 单段 5 秒 | 约 4 秒 | 约 0.000825 rad | 约 0.001125 rad | 约 0.003 rad·s |
| 单段 9 秒 | 约 8 秒 | 约 0.000412 rad | 约 0.0005625 rad | 约 0.003 rad·s |

对应原始数据：

- round_trip_T5s_20261020_111953_093511.csv
- round_trip_T9s_20261021_123838_521138.csv

在本次条件下，移动时间加倍后，
时间加权 RMSE 和最大误差约减半，IAE 基本不变。

较慢运动的瞬时误差较小，但误差持续时间更长，
因此累计绝对误差不一定同比下降。

各段实际采样覆盖边界略有不同。
尚未完成系统重复测试，不能据此宣称重复性或可靠性已验证。

其他五个关节主要保持姿态，
其较小误差不能证明这些关节的运动跟踪性能。

### 取消实验

| 实验 | 取消被接受 | 最终状态 | 观察最大速度 | 最大位置变化范围 |
|---|---|---|---:|---:|
| 主动取消 | True | CANCELED，5 | 0.00058171 rad/s | 0.00001047 rad |
| 等待超时后取消 | True | CANCELED，5 | 0.00046578 rad/s | 0.00000838 rad |

对应记录：

- cancel_motion_20261021_212445_268456.json
- cancel_motion_20261021_214837_294079.json

两次均观察了约 1 秒仿真时间，并满足本实验设定的：

- 最大绝对速度不超过 0.001 rad/s。
- 最大关节位置变化范围不超过 0.001 rad。

阈值是本实验检查标准，不是厂家性能标准。
取消不代表瞬间完全静止，也不是实机安全急停验证。
Action 状态为 CANCELED 时，不能因控制器结果码为 0 就判定到达原目标。

data 中保留调试阶段的失败记录，不计入通过案例。

## 当前限制

- 批量分析脚本的运行结果待确认。
- 未进行系统重复测试。
- 尚无批量运动执行和跨实验指标汇总流程。
- 尚未完成从全新环境安装开始的独立复现验证。
- 未实现 MoveIt 规划、碰撞检查、避障或抓取。
- 未接入 MPC、ADRC，也未自行开发底层控制器。
- 未进行实机或末端定位精度验证。