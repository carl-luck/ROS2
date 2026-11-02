# 可直接浏览的 P2 典型数据

从正式批次选取同一轮前馈＋P/MPC 的控制日志、指标和求解记录，以及 solver_failure、solver_timeout、request_crash 三项人工故障；另保留一次开发阶段 feedback_receive_gap_exceeded 的失败报告。

每个 JSON/CSV 保持原字节，来源与 SHA256 见 source_manifest.json。典型子集不能替代三次重复的统计；正式完整证据早已发布于上级 p2_six_evidence_20261031.tar.gz，三次统计见 ../../docs/RESULTS.md。request_crash 中请求进程的 running 状态可能因 SIGKILL 无法更新，应联合 guard/case 的停止结果解释。人工异常不冒充自然求解失败，开发失败不混入正式均值。
