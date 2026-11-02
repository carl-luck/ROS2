# P3 典型验收和失败记录

- tf_roundtrip：带时间戳点/位姿往返与缺失 frame、未来时间异常。
- negative_tests：不可达、碰撞目标、极小预算和明确标记的人工错误注入。
- moveit_cancel_failed：MoveIt 层取消超时，问题仍未解决。
- controller_cancel_passed：控制器层取消、终态与停稳通过，不替代 MoveIt 层验收。
- watchdog_failed：完整 P 首次运行时命令校验失败并停稳。原日志不足以确定具体拒绝分支，后续修正取时竞态并完善诊断。
- planning_sample_failed：规划返回成功但加密外部采样拒绝，不能执行。

来源与哈希见 source_manifest.json。完整成功 P/MPC 跟踪日志见 ../source；不删除或改写失败原始目录。本子集用于审阅，不是可直接执行的重放包。
