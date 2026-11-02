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
