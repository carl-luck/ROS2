# 模块 6 取消测试诊断

只读检查时间：2026-11-02（Asia/Shanghai）。

- 无运动异常测试通过：data/runs/20261101_235141+0800_failure_cases_19189559。
- MoveIt 取消测试未通过：data/runs/20261102_000510+0800_detour_plan_61b3fb60/execution_inputs_20261102_000540+0800_d8419fde/cancel_test_20261102_000551+0800_ee112b19。
- 客户端在约 2 秒仿真时间触发取消，等待应答超时，未确认最终取消状态，也未在等待窗口内确认停稳。
- MoveIt 日志随后记录准备轨迹完整执行成功，并记录取消和结果应答发送超时。不能将本次结果写成取消成功，根因仍待区分。
- 后续只读 /joint_states 单次采样位置接近固定绕障起点，六关节速度约 1e-13 rad/s；这表示该次采样时接近静止，不代表整个取消过程都已停稳。

新增独立脚本 scripts/test_controller_cancel.py：使用相同场景、控制器及起点核查，直接向 /joint_trajectory_controller/follow_joint_trajectory 发送已校验 detour 并测试取消。仅由用户手动运行，需要 --execute。取消测试不自动重试，不自动复位。

该诊断若通过，只能说明控制器 Action 路径的取消通过，不能替代 MoveIt ExecuteTrajectory 取消验收。保留旧失败报告。机器人在取消后停在中间位置，之后执行必须重新检查起点并重新准备轨迹。
