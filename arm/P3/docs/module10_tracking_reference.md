# 模块 10：冻结 P/MPC 共用参考轨迹

目标是完成同轨迹数据准备，不进行控制器切换或在线运动。P1/P2 保持不变。

只读核查发现：P2 六关节积分器 MPC 的 Ts=0.02 s、horizon=25，基线为参考速度前馈＋kp=2.0 的位置 P；速度限值每关节 0.02 rad/s，命令变化率 0.1 rad/s²。参考为内置周期函数，并有小幅位移与墙钟超时限制。P2 碰撞世界预检查只支持机器人和地面，不能直接覆盖 P3 箱体。

本模块选取之前重定时、样条采样并实际执行通过的原始绕障段作为 common reference，不挑选模块 8 最快或最短样本。通过统一时间缩放保持其规范化五次多项式几何形状，使参考速度不超过 0.018 rad/s、加速度不超过 0.09 rad/s²，为未来 0.02/0.1 命令限值留出部分余量。余量不保证跟踪控制一定可行。

Ubuntu Remote-SSH 终端，目录 ~/ROS2/arm/P3，不需要 source P2 工作空间或激活其 venv：

```bash
cd ~/ROS2/arm/P3
python3 scripts/prepare_tracking_reference.py \
  --inputs-dir "$HOME/ROS2/arm/P3/data/runs/20261101_230017+0800_detour_plan_b493ef44/execution_inputs_20261101_231430+0800_0f87bf74"
```

输出 passed=true，robot_motion_requested=false，execution_ready=false。后一个 false 是刻意标记：还未完成实时状态/场景/控制器核查与在线适配。

新目录含 reference.csv（0.02 s 观察用采样）、reference_polynomials.npz（正式共同参考参数）、模型快照、源轨迹、导出源码和 tracking_reference_report.json。源文件和导出物有 SHA256。未来 P 与 MPC 都调用保存的相同多项式 evaluator，使用相同起点、时间尺度和约束；不能各自重采样出不同路径。

离线检查包括源证据与哈希、关节顺序、端点零速度、数值速度/加速度极值、位置连续性及有限差分导数检查。时间缩放不新增几何路径，但历史碰撞检查不能替代当前场景验收；在线跟踪误差仍可能导致碰撞。

按当前轨迹峰值速度推算约 189.6 s 仿真时长。未来在线程序必须据此设置超时、完整路径关节界限与障碍监控，不能直接放宽或绕过 P2 原 guard。

验收后先做 P3 独立离线 P/MPC 同参考回放，检查求解状态、限幅、误差与日志；完成之后再安排人工控制器切换和 Gazebo 小步验证。本模块的 CSV 不可直接作为速度命令发送。
