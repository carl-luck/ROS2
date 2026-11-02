# Gazebo 在线同轨迹跟踪整包

本模块由用户在 Ubuntu Remote-SSH 终端手动执行。需要已有 Gazebo、MoveIt、箱体场景持续运行；不要重启第二套仿真，也不要在 RViz 发送运动。所有新增文件均在 P3，P1/P2 未改。接口/语法检查已完成，在线运动试跑尚待本模块验收。

P3 的起点准备工具最多保存 3 次开发规划尝试，找到通过结果后进行重定时与样条采样校验，再只执行准备段。准备失败不执行后续跟踪；不能把这些带重试的准备过程当成规划成功率评测。

## 1. 同一个终端加载环境与工具

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash
source ~/ROS2/arm/P3/ros2_ws/install/setup.bash
cd ~/ROS2/arm/P3
export P3_TRACK_REF="$HOME/ROS2/arm/P3/data/runs/20261102_202248+0800_tracking_reference_c270af24"
unset P3_PILOT_DIR P3_P_DIR P3_MPC_DIR

p3_capture() {
  local dest="$1" report="$2" log rc saved
  shift 2
  unset "$dest"
  log="$(mktemp /tmp/p3_online_XXXXXXXX.txt)"
  "$@" 2>&1 | tee "$log"
  rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ]; then
    printf '本步失败，停止后续步骤。日志：%s\n' "$log"
    return "$rc"
  fi
  saved="$(sed -n 's/^SAVED: //p' "$log" | tail -n 1)"
  python3 -c 'import json,sys; from pathlib import Path; assert json.loads((Path(sys.argv[1])/sys.argv[2]).read_text())["passed"] is True' "$saved" "$report" || return 2
  printf -v "$dest" '%s' "$saved"
  export "$dest"
  printf '本步通过：%s=%s\n' "$dest" "$saved"
}

p3_position() {
  ros2 control switch_controllers --deactivate forward_velocity_controller \
    --activate joint_trajectory_controller --strict
}
p3_velocity() {
  ros2 control switch_controllers --deactivate joint_trajectory_controller \
    --activate forward_velocity_controller --strict
}
p3_prepare() {
  p3_capture P3_START_DIR tracking_start_report.json \
    python3 scripts/prepare_tracking_start.py --execute
}
p3_track() {
  local dest="$1" mode="$2"
  shift 2
  p3_capture "$dest" gazebo_tracking_report.json \
    "$HOME/ROS2/arm/P2/.venv/bin/python" scripts/run_tracking_gazebo.py \
    --reference-dir "$P3_TRACK_REF" --config config/tracking_offline_v1.json \
    --mode "$mode" "$@" --execute
}
```

## 2. 只加载速度控制器，暂不激活

初次执行下面命令；若已经加载，先查看 list_controllers，不要反复 spawner。期望 joint_trajectory_controller active、forward_velocity_controller inactive。

```bash
ros2 run controller_manager spawner forward_velocity_controller \
  --inactive --controller-type velocity_controllers/JointGroupVelocityController \
  --param-file "$HOME/ROS2/arm/P3/config/tracking_velocity_controller.yaml"

ros2 control list_controllers
```

若初始状态已是速度控制器 active，先 p3_position，再准备起点。若状态不符或仍有机器人运动，停止，不继续试跑。

## 3. 起点准备、5 秒 P 试跑、恢复位置控制器

下面会运动。链式命令任何一步失败即停止该组后续动作，失败后不要执行下一组。

```bash
p3_prepare && p3_velocity && \
  p3_track P3_PILOT_DIR p --pilot-seconds 5 && p3_position
```

验收：tracking_start_report.json 与 gazebo_tracking_report.json 均 passed=true，stopped_confirmed=true，切换恢复成功。试跑仍停在中途，完整测试前必须重新准备起点。

## 4. 完整前馈＋P

```bash
test -n "${P3_PILOT_DIR:?先完成试跑}" && \
  p3_prepare && p3_velocity && p3_track P3_P_DIR p && p3_position
```

参考时长约 189.58 s 仿真时间，末尾观察 2 s；虚拟机墙钟时间可能明显更久。保持终端运行，期间不手动移动机械臂，不运行其他跟踪程序。

## 5. 完整 MPC

```bash
test -n "${P3_P_DIR:?先完成完整P测试}" && \
  p3_prepare && p3_velocity && p3_track P3_MPC_DIR mpc && p3_position
```

同一参考、初态容差、限值与监控标准。MPC 计算超 20 ms 时使用本周期前馈＋P 退化命令并记录；OSQP 求解失败回退也单独统计。求解/通信阻塞导致请求超时则监护进程发零。

## 6. 汇总

```bash
p3_capture P3_GAZEBO_COMPARE_DIR gazebo_comparison_report.json \
  "$HOME/ROS2/arm/P2/.venv/bin/python" scripts/summarize_gazebo_tracking.py \
  --p-dir "${P3_P_DIR:?缺少通过的P测试}" \
  --mpc-dir "${P3_MPC_DIR:?缺少通过的MPC测试}"
```

将最后 SAVED 目录发回。任一步失败则只反馈该步 SAVED 与错误，不跳过失败、不重发旧轨迹。下一次尝试必须先核对实际控制器和机械臂状态。

## 在线监控与边界

速度监护进程是命令话题的正常唯一发布者。检查反馈新鲜度、命令时戳与心跳、实际速度/关节范围及重复命令发布者；命令超过 0.25 s 墙钟未更新则发零。跟踪主进程定期检查实际状态、0.1 s 常命令外推状态、MoveIt 场景与控制器归属，服务响应超过 0.5 s 则停止。状态采样不是连续防碰撞，MPC 没有内置障碍约束。

正常/异常结束都发零并要求速度小于等于 0.001 rad/s 持续 0.5 s 仿真时间。停止零命令绕过变化率限制。双进程并不是硬件安全系统，两个进程同时崩溃仍可能留下最后的速度命令；如果 stopped_confirmed=false、程序卡住或机械臂持续运动，在 Gazebo 启动终端按 Ctrl+C 并保留所有日志。不要启动第二个速度发布程序来试图覆盖故障。

Gazebo 箱体与 MoveIt 对齐只在开始双端查询核查，在线主要监督 MoveIt 场景；不支持外部在 Gazebo 中悄悄移动障碍的实验。两种控制器各一次完整运行是开发对比，不能当作统计可靠性结论。
