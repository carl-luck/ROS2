# P3 坐标系与 TF2 实践记录

更新日期：2026-11-02。原始纸笔知识通过实际 ROS2 消息转换与日志进行了验证。

## 已核查关系

- SRDF 规划组 ur_manipulator，链 base_link → tool0；URDF 模型根 world，P3 显式使用 world 表达规划目标。
- world 与 base_link 在当前安装配置下为单位变换；base_link ← base 平移为零、绕 Z 旋转 180°，不能混用 base 与 base_link 的坐标数字。
- 上述模型根与显式任务参考系不等同于已调用 MoveGroupInterface::getPlanningFrame()；后者尚未独立核查。当前 RViz Fixed Frame 的 GUI 值也未在此单独验收。
- Gazebo world 与 TF world 的映射来自明确的安装约定，并核对机器人模型放置、URDF 与 TF；不是由同名自动建立。

## 已完成实践

使用 Python Buffer/TransformListener 查询已有 TF，将同一时间戳的 PointStamped 与 PoseStamped 从 tool0 转到 world，再使用反向变换转回。教学点在 tool0 中为 [0.02, 0, 0] m；某次保存结果在 world 中约为 [0.008815, 0.233663, 1.079371] m，该数值依赖当时机械臂姿态，不能当固定安装参数。

往返位置误差约 6.7e-16 m、姿态角误差约 9.6e-16 rad，属于数值一致性检查，不代表实机精度。源姿态单位四元数转换到 world 后一般不再是单位四元数。

查询方向：lookup_transform(target_frame, source_frame, time) 返回用于将 source 数据转换到 target 的关系；平移表示 source 原点在 target 中的位置。消息 frame_id 和 stamp 必须与数据所属坐标系和时刻一致。

缺失 frame 查询得到 LookupException；未来时间查询得到 ExtrapolationException。时间问题优先核对 /clock、use_sim_time、消息时间戳与 TF 缓存，不为消除错误随意补静态 TF。机器人状态由关节反馈与模型发布，实验未向机器人树重复发布关系。

## 证据与尚待补充

原始报告：evidence/tf/20261101_152042+0800_stamped_transform_d0305591/tf_report.json；消息与变换：同目录 transforms_and_messages.json。环境树核查位于 evidence/tf/20261031_180113+0800_module01_xSCxL0us/。

已掌握范围可表述为：实际 TF 查询、带 frame_id/stamp 的点与位姿转换、同时间戳往返核验、缺失 frame 与未来外推诊断。断开 TF 树的受控实践、缓存过期与历史数据时间查询的独立验收仍待补充，不能笼统宣称所有 TF 故障诊断均已完成。
