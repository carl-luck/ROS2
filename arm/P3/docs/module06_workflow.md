# P3 模块 6：异常处理与执行取消

目标：保存不可达目标、碰撞目标、极小规划预算、人工错误注入和真实 Action 取消的证据。人工注入与真实规划结果分别记录，不计入正常成功率。

全程在同一个 Ubuntu Remote-SSH Bash 终端执行。Gazebo、MoveIt 保持运行；保持箱体不变，期间不要通过 RViz 或其他脚本发送运动。无须安装或构建。任一步失败，停止后续步骤并保留 SAVED 目录。所有规划和执行由使用者亲自运行。

## 1. 环境与日志工具

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/arm/P1/ros2_ws/install/setup.bash
source ~/ROS2/arm/P3/ros2_ws/install/setup.bash
cd ~/ROS2/arm/P3

export P3_CASE_DIR="$HOME/ROS2/arm/P3/data/runs/20261101_224558+0800_detour_search_wELTEptP"
export P3_OLD_PLAN="$HOME/ROS2/arm/P3/data/runs/20261101_230017+0800_detour_plan_b493ef44"

p3_capture() {
  local dest="$1" report="$2" log rc saved
  shift 2
  unset "$dest"
  log="$(mktemp /tmp/p3_module06_XXXXXXXX.txt)"
  "$@" 2>&1 | tee "$log"
  rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ]; then
    printf '本步失败，停止后续步骤。控制台日志：%s\n' "$log"
    return "$rc"
  fi
  saved="$(sed -n 's/^SAVED: //p' "$log" | tail -n 1)"
  python3 -c 'import json,sys; from pathlib import Path; assert json.loads((Path(sys.argv[1])/sys.argv[2]).read_text())["passed"] is True' "$saved" "$report" || return 2
  printf -v "$dest" '%s' "$saved"
  export "$dest"
  printf '本步通过：%s=%s\n' "$dest" "$saved"
}
```

## 2. 无运动异常测试

```bash
p3_capture P3_FAILURE_DIR failure_report.json \
  python3 scripts/run_failure_cases.py \
  --plan-dir "$P3_OLD_PLAN" \
  --config config/failure_cases_v1.json
```

验收：passed=true，execution_requests_sent=0；远目标 IK 被拒绝，碰撞目标被检出，两项人工注入被拒绝。极小预算可能规划成功，保留实际结果，不强求超时。

## 3. 从当前状态重新规划与校验

旧轨迹起点已经过期，不直接重发。下列两条命令不执行运动。

```bash
test -n "${P3_FAILURE_DIR:?先完成第2步}"
p3_capture P3_CANCEL_PLAN detour_plan_report.json \
  python3 scripts/plan_detour_case.py \
  --case-dir "$P3_CASE_DIR" \
  --config config/detour_plan_v1.json
```

```bash
p3_capture P3_CANCEL_INPUTS execution_inputs_report.json \
  python3 scripts/prepare_detour_execution.py \
  --plan-dir "${P3_CANCEL_PLAN:?先完成重新规划}" \
  --config config/detour_retime_v1.json
```

验收：两次 passed=true。校验包括重新定时后的控制器样条、速度/加速度及保存场景中的离散碰撞采样。采样通过不等于连续碰撞保证。

## 4. 真实执行取消测试：本步会运动

```bash
p3_capture P3_CANCEL_DIR sequence_report.json \
  python3 scripts/test_execution_cancel.py \
  --inputs-dir "${P3_CANCEL_INPUTS:?先完成轨迹校验}" \
  --config config/detour_execute_v1.json \
  --cancel-after-sim-s 2.0 \
  --execute
```

仅执行 preparation，在 Action 接受后的约 2 秒仿真时间触发取消，不发送后续 detour。仿真时间不是墙钟时间，虚拟机中可能更久。若轨迹过短则在发送前拒绝；保留报告，不修改阈值绕过检查。

验收：passed=true、scheduled_cancel_triggered=true、cancel_acknowledged=true、final_action_status=5、stopped_after_failure=true、post_cancel_state_valid=true、post_cancel_scene_unchanged=true。取消应有服务端应答、Action 终态和实际反馈停稳三类证据。

若取消或停稳无法确认，不再发运动；若仿真仍在运动，在 Gazebo 启动终端按 Ctrl+C。保留日志。这是仿真 Action 取消测试，不代表独立急停系统。

## 5. 离线汇总

```bash
p3_capture P3_MODULE06_DIR module06_report.json \
  python3 scripts/summarize_module06.py \
  --failure-dir "${P3_FAILURE_DIR:?先完成异常测试}" \
  --cancel-dir "${P3_CANCEL_DIR:?先完成取消测试}"
```

验收：module06_report.json 的 passed=true，checks 全为 true。反馈最终 SAVED 路径。失败时反馈该步 SAVED 路径和错误即可。

取消后机器人留在中途停稳位置；禁止重发本次轨迹，后续执行重新规划和校验。之后进入固定场景重复规划评测、指标汇总与项目文档。P 与 MPC 同轨迹跟踪为后续阶段。
