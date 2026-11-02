# 复现说明与发布边界

本目录为 P3 发布候选，包含源码、版本化配置、关键报告与图表源数据。没有在干净虚拟机完成从零安装验收，因此不是开箱即用的独立发行包。

## 已使用环境

Ubuntu 22.04、ROS2 Humble、Python 3.10.12、Gazebo Fortress/Gazebo Sim 6、MoveIt 2、UR ROS2 Gazebo 与 MoveIt 配置。离线求解环境版本为 numpy 1.21.5、scipy 1.8.0、osqp 1.0.4；图表使用 matplotlib 3.5.1。确切已安装 ROS 包版本见 results/environment_packages.txt（若提供）。

Python 依赖包括 numpy、scipy、osqp、matplotlib、PyYAML、protobuf；ROS 消息与 TF 库通过 ROS 安装提供，不建议用 pip 替代 rclpy、tf2、moveit_msgs 等系统包。本项目未自动安装或升级依赖。

现有命令引用 ~/ROS2/arm/P1/ros2_ws/install/setup.bash（已安装 UR 配置）和 ~/ROS2/arm/P2/.venv/bin/python（OSQP）。复现者可以提供等价的独立环境，但不能假设仅克隆 P3 即拥有这些依赖。候选包不包含 P1/P2 的源码、虚拟环境或构建产物。

## 最快查看结果：不启动仿真

- results/source/：关键原始 JSON、规划 20 次明细及两次在线逐步日志。
- results/figures/：PNG、SVG、PDF 与图表说明/数据哈希。
- docs/gazebo_tracking_results.md：在线结果及起点差异、超预算退化等限制。
- docs/offline_tracking_results.md：离线模型对比，不能混成在线测量。

重画图表（已有 matplotlib/numpy 的 Python 中）：

```bash
python3 scripts/make_project_figures.py --root .
```

此命令只读结果数据并重写图表，不访问 ROS 或发送运动。图表为项目展示格式，不针对某一期刊规范。

## 原始证据与运行

data/ 与 evidence/ 含开发期大批原始日志及模型快照；media/ 下 MP4 默认不提交。候选压缩包保留本地原始数据供审阅，而 Git 候选清单只包含精简源码/结果。原始 data 路径不是远程下载地址；未上传任何平台。运行指令与目录对应关系见 docs/evidence_index.md 和各模块文档。

固定候选中的绝对路径、JSON case_dir 和证据路径来自原实验机器。迁移机器必须核查并重新生成对应场景/轨迹证据；不能把带旧绝对路径的执行输入当成可直接重放文件。模型、网格或控制器版本变化也需重新验证。

工具包人工构建入口（只在依赖齐备、用户自行执行时使用）：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash
cd ros2_ws
colcon build --packages-select ur5e_planning_eval
source install/setup.bash
```

该构建步骤未在交付整理阶段重新运行；保留之前用户构建和验收状态。ROS 控制器切换、规划请求、场景修改与运动脚本均由用户手动执行。完整流程见 docs/module12_gazebo_tracking.md，先短段、后完整运行，旧轨迹不能越过起点/场景检查。

## 已知问题

1. MoveIt ExecuteTrajectory 取消超时尚未解决；控制器 FollowJointTrajectory 取消通过，二者不同。
2. 跟踪 MPC 预测运动学积分模型，无显式障碍约束；离散实际状态监控不是连续防碰撞。
3. 在线 MPC 发生 3 次计算超 20 ms 后退化为前馈＋P，不保证硬实时。
4. 在线 P/MPC 各仅一次完整通过，初态不完全一致且轨迹低速；结论仅限本次仿真。
5. 项目启动和少量开发脚本依赖实验机目录；干净环境一键复现尚未验收。
6. package.xml 含 carl@example.com 维护者占位地址；公开发布前由作者填写有效联系信息。package.xml 声明 Apache-2.0，但此整理未增加授权文件或替作者决定全部内容的许可；发布前核对仓库许可与第三方模型/数据来源。

## 上传前人工检查

查看候选清单，确认希望公开的数据范围、作者联系方式与许可。不要直接 git add 整个原始 data；确认忽略规则之后再暂存选定文件。此交付未 commit、push、创建 Release 或上传视频。
