# 离线对比整包：前馈＋P 与六关节 MPC

这是离线软件回放，不连接 ROS、不切换控制器、不执行机器人运动。P1/P2 文件未修改，仅使用 P2 已有 venv 中的 numpy/scipy/osqp。P3 新增 tracking_control_core.py、run_tracking_offline.py、config/tracking_offline_v1.json。

## 单命令操作

Ubuntu Remote-SSH 终端，仿真开关均不影响此实验，无须 source 工作空间或激活 venv：

```bash
cd ~/ROS2/arm/P3
~/ROS2/arm/P2/.venv/bin/python scripts/run_tracking_offline.py \
  --reference-dir "$HOME/ROS2/arm/P3/data/runs/20261102_202248+0800_tracking_reference_c270af24" \
  --config config/tracking_offline_v1.json
```

预计几十秒到数分钟，189.58 秒参考时间由数值循环推进，不是要求墙钟等待四次完整时长。运行时显示 ideal_p、ideal_mpc、lag_delay_p、lag_delay_mpc。不要同时运行其他计算密集型实验，以免干扰墙钟计算时间。

## 固定条件

- 共同参考为已冻结的同一份多项式和同一起点，先核查哈希。
- Ts=0.02 s、MPC 预测 25 步；P 为速度前馈＋kp=2 的位置误差反馈。
- 同一命令速度限值 0.02 rad/s、变化率 0.1 rad/s²，关节界限采用 URDF/soft limit 并留 0.05 rad。
- MPC 继承 P2 的运动学积分模型及权重 q=100、v=1、du=0.1；改为采样共同参考，不再用 P2 内置周期参考和小幅 excursion 窗口。
- ideal 为 q_next=q+Ts*u；lag_delay 为 1 步命令延迟和 0.08 s 一阶速度滞后。两控制器面对相同模型，MPC 在第二种模型中仍预测理想积分器，属于明确的模型失配实验。
- 每组参考结束后观察 2 秒收敛；无随机噪声、无扭矩动力学。虚拟模型不检查实际跟踪路径碰撞。

## 输出与验收

目录包含 offline_report.json、metrics.csv、四组逐步 CSV，以及配置/源码快照和参考哈希。运行过程日志记录误差、命令、模拟速度、求解状态、限幅修正与计算时间。

passed=true 仅表示四组运行完成、数值有限、命令及关节范围检查通过；不表示 MPC 更优、在线实时或碰撞安全。另行检查 fallback_count、compute_budget_overrun_count，不能只看 passed。求解失败回退到前馈＋P，回退会显式统计。超过 20 ms 的计算耗时会统计，本离线循环没有真实 ROS 调度，不据此承诺实时性能。

tracking_rmse_rad 是参考运动期间所有关节和时间样本的均方根误差；full_run_rmse_rad 包含停止后的观察期。计算时间只覆盖参考计算、控制求解和命令限幅，排除 ROS、状态监控和文件写入。CSV 中命令是在该行状态下算出的下一步输入，状态/参考误差按同一时刻对齐。

验收后将 SAVED 目录反馈，再分析四组结果并准备 Gazebo 在线适配。进入在线前仍需完整箱体碰撞监控、实际起点准备、人工切换位置/速度控制器、失败停机与看门狗策略。禁止将 CSV 直接发布成速度指令。
