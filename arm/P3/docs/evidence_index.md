# P3 第一阶段证据索引

路径相对 arm/P3；原始数据目前在本地虚拟机，未承诺上传 GitHub。复制源码时应另行提供数据获取方式。失败目录保留，不以成功目录替换。

| 内容 | 本地证据路径 |
|---|---|
| TF 环境核查 | evidence/tf/20261031_180113+0800_module01_xSCxL0us/ |
| 带时间戳点、位姿往返及错误诊断 | evidence/tf/20261101_152042+0800_stamped_transform_d0305591/tf_report.json |
| TF 数值原始消息 | evidence/tf/20261101_152042+0800_stamped_transform_d0305591/transforms_and_messages.json |
| 初始位姿 IK 失败 | data/runs/20261101_215854+0800_pose_plan_c61e0583/ |
| FK 构造位姿任务 | data/runs/20261101_221040+0800_pose_plan_9ef49eda/ |
| 箱体双端核查 | data/runs/20261101_222932+0800_box_verify_c8bc257d/scene_report.json |
| 固定绕障候选、直达碰撞 | data/runs/20261101_224558+0800_detour_search_wELTEptP/ |
| 绕障规划 | data/runs/20261101_230017+0800_detour_plan_b493ef44/ |
| 重定时与样条检查 | data/runs/20261101_230017+0800_detour_plan_b493ef44/execution_inputs_20261101_231430+0800_0f87bf74/ |
| 实际准备＋绕障执行通过 | data/runs/20261101_230017+0800_detour_plan_b493ef44/execution_inputs_20261101_231430+0800_0f87bf74/sequence_20261101_232914+0800_23180fb9/sequence_report.json |
| 无运动异常测试通过 | data/runs/20261101_235141+0800_failure_cases_19189559/failure_report.json |
| MoveIt 取消失败 | data/runs/20261102_000510+0800_detour_plan_61b3fb60/execution_inputs_20261102_000540+0800_d8419fde/cancel_test_20261102_000551+0800_ee112b19/sequence_report.json |
| 控制器取消通过 | data/runs/20261102_000510+0800_detour_plan_61b3fb60/execution_inputs_20261102_000540+0800_d8419fde/controller_cancel_test_20261102_115619+0800_3f253211/sequence_report.json |
| 模块 7 十次评测 | data/runs/20261102_193604+0800_planning_batch_OTy0LbiX/batch_report.json |
| 模块 8 对照实验 | data/runs/20261102_200323+0800_resolution_comparison_e91af440/comparison_report.json |
| 模块 8 全部试次 | data/runs/20261102_200323+0800_resolution_comparison_e91af440/trials.csv |
| 视频来源与哈希 | media/video_manifest.json |

视频副本为 media/20261101_ur5e_detour_demo_original.mp4，SHA256 为 67e533839dff773853c0a56da99263e766fa538c7c60c735687c7e1c52606ae6。未转码，未核实与某一次日志的逐帧对应；不以录屏替代数据验收。

## 同轨迹跟踪补充证据

- 共同参考：data/runs/20261102_202248+0800_tracking_reference_c270af24/tracking_reference_report.json。
- 离线四组：data/runs/20261102_202849+0800_tracking_offline_c52e9e11/offline_report.json。
- 完整 P：data/runs/20261102_210430+0800_gazebo_tracking_p_ca08cebd/gazebo_tracking_report.json。
- 完整 MPC：data/runs/20261102_211826+0800_gazebo_tracking_mpc_022babee/gazebo_tracking_report.json。
- 在线对比：data/runs/20261102_212720+0800_gazebo_comparison_e3f1d5fb/gazebo_comparison_report.json。
- 原失败保留：data/runs/20261102_204758+0800_gazebo_tracking_p_b3bef7d3/gazebo_tracking_report.json。
