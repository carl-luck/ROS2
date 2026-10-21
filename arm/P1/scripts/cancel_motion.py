import argparse
import json
import math
from datetime import datetime
from pathlib import Path
from time import monotonic

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectoryPoint

from round_trip import RoundTrip


class CancelExperiment(RoundTrip):
    def __init__(self):
        super().__init__(9)
        self.velocities = None

    def receive_state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))

        if not all(
            name in positions and name in velocities
            for name in self.joints
        ):
            return

        position_values = [
            positions[name]
            for name in self.joints
        ]

        velocity_values = [
            velocities[name]
            for name in self.joints
        ]

        if not all(
            math.isfinite(value)
            for value in position_values + velocity_values
        ):
            return

        self.positions = position_values
        self.velocities = velocity_values

        self.state_stamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )

    def wait_for(self, condition, timeout):
        deadline = monotonic() + timeout

        while rclpy.ok():
            if condition():
                return True

            remaining = deadline - monotonic()

            if remaining <= 0:
                return False

            rclpy.spin_once(
                self,
                timeout_sec=min(0.1, remaining),
            )

        return False

    def feedback(self, message):
        data = message.feedback

        if 'shoulder_pan_joint' not in data.joint_names:
            return

        index = data.joint_names.index('shoulder_pan_joint')

        if len(data.actual.positions) <= index:
            return

        if monotonic() - self.last_print >= 0.5:
            print(
                f'运动反馈：底座='
                f'{data.actual.positions[index]:.6f} rad',
                flush=True,
            )

            self.last_print = monotonic()


def main():
    parser = argparse.ArgumentParser(
        description='机械臂主动取消与客户端等待超时实验'
    )

    parser.add_argument(
        '--mode',
        choices=['active', 'timeout'],
        default='active',
    )

    args = parser.parse_args()
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = CancelExperiment()

    report = {
        'schema_version': 2,
        'experiment': (
            'active_cancel'
            if args.mode == 'active'
            else 'client_result_timeout_cancel'
        ),
        'mode': args.mode,
        'client_result_wait_timeout_wall_s': (
            5.0 if args.mode == 'timeout' else None
        ),
        'client_result_wait_timed_out': False,
        'joint_names': node.joints,
        'target_pan_rad': 0.30,
        'trajectory_duration_sim_s': 9,
        'first_point_time_sim_s': 1,
        'cancel_delay_from_acceptance_sim_s': (
            3 if args.mode == 'active' else None
        ),
        'goal_accepted': None,
        'cancel_request_sent': False,
        'cancel_response_received': False,
        'cancel_return_code': None,
        'cancel_accepted': None,
        'final_action_status': None,
        'controller_error_code': None,
        'controller_error_string': None,
        'stop_observation': None,
        'stop_samples': [],
        'experiment_passed': False,
        'failure_reason': None,
        'cleanup_error': None,
        'cleanup_final_action_status': None,
    }

    output = None
    stop_samples = report['stop_samples']

    try:
        folder = Path(__file__).resolve().parent.parent / 'data'
        folder.mkdir(parents=True, exist_ok=True)

        output = folder / (
            f'cancel_motion_{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        )

        start = node.fresh_state()
        limits = node.read_limits()

        target = start.copy()
        target[0] = 0.30

        change = abs(target[0] - start[0])

        if change < 0.05:
            raise RuntimeError(
                '当前位置太接近 0.30 rad，本次不发送目标。'
            )

        if change > 0.35:
            raise RuntimeError(
                '底座变化超过本实验允许的 0.35 rad。'
            )

        for name, value in zip(node.joints, target):
            lower, upper = limits[name]

            if not lower <= value <= upper:
                raise RuntimeError(
                    f'{name} 目标超出模型限位。'
                )

        if not node.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                '未找到轨迹 Action 服务端。'
            )

        report['start_positions_rad'] = start.copy()
        report['target_positions_rad'] = target.copy()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = node.joints

        for positions, duration in [
            (start, 1),
            (target, 9),
        ]:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6
            point.time_from_start.sec = duration

            goal.trajectory.points.append(point)

        print(
            f'底座 {start[0]:.6f} → 0.300000 rad，'
            '计划中途取消。'
        )

        node.send_future = node.client.send_goal_async(
            goal,
            feedback_callback=node.feedback,
        )

        if not node.wait_for(node.send_future.done, 5.0):
            raise RuntimeError(
                '目标响应超时，接受情况未知。'
            )

        node.goal_handle = node.send_future.result()
        report['goal_accepted'] = node.goal_handle.accepted

        if not node.goal_handle.accepted:
            raise RuntimeError('轨迹被拒绝。')

        node.result_future = node.goal_handle.get_result_async()

        accepted_sim = (
            node.get_clock().now().nanoseconds * 1e-9
        )

        report['goal_response_observed_sim_time_s'] = accepted_sim

        if args.mode == 'active':
            def cancel_time_reached():
                now = (
                    node.get_clock().now().nanoseconds * 1e-9
                )

                if now < accepted_sim:
                    raise RuntimeError(
                        '等待期间仿真时间倒退，请检查是否重置仿真。'
                    )

                return (
                    node.result_future.done()
                    or now - accepted_sim >= 3.0
                )

            if not node.wait_for(cancel_time_reached, 20.0):
                raise RuntimeError(
                    '等待取消时机超时；'
                    '检查仿真是否暂停或运行过慢。'
                )

        else:
            print(
                '客户端开始等待结果，'
                '上限为 5 秒墙钟时间。'
            )

            wait_started = monotonic()

            completed = node.wait_for(
                node.result_future.done,
                5.0,
            )

            report['client_result_wait_observed_wall_s'] = (
                monotonic() - wait_started
            )

            report['sim_elapsed_during_result_wait_s'] = (
                node.get_clock().now().nanoseconds * 1e-9
                - accepted_sim
            )

            if not completed:
                report['client_result_wait_timed_out'] = True

                print(
                    '客户端等待结果超时，'
                    '准备自动取消；这不表示控制器故障。'
                )

        if node.result_future.done():
            result = node.result_future.result()

            report['final_action_status'] = result.status
            report['controller_error_code'] = (
                result.result.error_code
            )
            report['controller_error_string'] = (
                result.result.error_string
            )

            raise RuntimeError(
                '请求取消前任务已经结束，'
                '本次没有完成预期的取消实验。'
            )

        report['cancel_request_sim_time_s'] = (
            node.get_clock().now().nanoseconds * 1e-9
        )

        print('现在发送取消请求。')

        cancel_future = node.goal_handle.cancel_goal_async()
        report['cancel_request_sent'] = True

        if node.wait_for(cancel_future.done, 5.0):
            response = cancel_future.result()

            report['cancel_response_received'] = True
            report['cancel_return_code'] = int(
                response.return_code
            )

            report['cancel_accepted'] = any(
                bytes(item.goal_id.uuid)
                == bytes(node.goal_handle.goal_id.uuid)
                for item in response.goals_canceling
            )

            print(
                f'取消请求被接受：'
                f'{report["cancel_accepted"]}'
            )

        else:
            print(
                '尚未收到取消响应，继续等待最终结果。'
            )

        if not node.wait_for(node.result_future.done, 15.0):
            raise RuntimeError(
                '未确认最终状态，不能认为任务已经停止。'
            )

        result = node.result_future.result()

        report['final_action_status'] = result.status
        report['controller_error_code'] = (
            result.result.error_code
        )
        report['controller_error_string'] = (
            result.result.error_string
        )
        report['final_result_observed_sim_time_s'] = (
            node.get_clock().now().nanoseconds * 1e-9
        )

        print(
            f'最终 Action 状态={result.status}，'
            f'控制器结果码={result.result.error_code}'
        )

        node.fresh_state()

        if node.velocities is None:
            raise RuntimeError(
                '没有完整速度反馈，不能检查是否停止。'
            )

        observation_start = node.state_stamp
        previous_stamp = None

        print('开始观察约 1 秒仿真时间内的关节速度和位置。')

        def collect_stop_state():
            nonlocal previous_stamp

            stamp = node.state_stamp

            if stamp is None or node.velocities is None:
                return False

            if stamp < observation_start:
                raise RuntimeError(
                    '观察期间仿真时间倒退。'
                )

            if (
                previous_stamp is not None
                and stamp < previous_stamp
            ):
                raise RuntimeError(
                    '关节状态时间戳倒退。'
                )

            if stamp != previous_stamp:
                previous_stamp = stamp

                stop_samples.append({
                    'sim_time_s': stamp,
                    'positions_rad': node.positions.copy(),
                    'velocities_rad_s': node.velocities.copy(),
                })

            return (
                len(stop_samples) >= 2
                and stamp - observation_start >= 1.0
            )

        if not node.wait_for(collect_stop_state, 10.0):
            raise RuntimeError(
                '未收集到覆盖 1 秒仿真时间的停止观察数据；'
                '检查仿真是否暂停、状态是否持续发布。'
            )

        position_range = max(
            max(
                sample['positions_rad'][i]
                for sample in stop_samples
            )
            - min(
                sample['positions_rad'][i]
                for sample in stop_samples
            )
            for i in range(6)
        )

        max_speed = max(
            abs(value)
            for sample in stop_samples
            for value in sample['velocities_rad_s']
        )

        stop_passed = (
            max_speed <= 0.001
            and position_range <= 0.001
        )

        report['stop_observation'] = {
            'observed_span_sim_s': (
                stop_samples[-1]['sim_time_s']
                - stop_samples[0]['sim_time_s']
            ),
            'samples': len(stop_samples),
            'max_abs_velocity_rad_s': max_speed,
            'max_joint_position_range_rad': position_range,
            'velocity_threshold_rad_s': 0.001,
            'position_range_threshold_rad': 0.001,
            'passed': stop_passed,
        }

        trigger_passed = (
            args.mode == 'active'
            or report['client_result_wait_timed_out'] is True
        )

        report['experiment_passed'] = (
            trigger_passed
            and report['cancel_accepted'] is True
            and result.status == GoalStatus.STATUS_CANCELED
            and stop_passed
        )

        print(
            f'停止观察：最大速度={max_speed:.8f} rad/s，'
            f'最大位置变化范围={position_range:.8f} rad'
        )

        print(
            f'观察样本数={len(stop_samples)}，'
            f'覆盖仿真时间='
            f'{report["stop_observation"]["observed_span_sim_s"]:.3f}s'
        )

        if report['experiment_passed']:
            print(f'本次 {args.mode} 实验检查通过。')
        else:
            report['failure_reason'] = (
                '取消响应、最终状态或停止观察'
                '未满足本实验检查条件。'
            )

            print('本次检查未通过，请保留结果分析。')

    except KeyboardInterrupt:
        report['failure_reason'] = '客户端收到 Ctrl+C'
        print('收到 Ctrl+C，尝试清理未完成目标。')

    except Exception as error:
        report['failure_reason'] = str(error)
        print(f'实验未通过：{error}')

    finally:
        try:
            node.cancel_pending()

        except Exception as error:
            report['cleanup_error'] = str(error)
            print(f'清理未完成目标时发生错误：{error}')

        finally:
            try:
                if (
                    node.result_future is not None
                    and node.result_future.done()
                    and not node.result_future.cancelled()
                ):
                    try:
                        cleanup_result = (
                            node.result_future.result()
                        )

                        report['cleanup_final_action_status'] = (
                            cleanup_result.status
                        )

                    except Exception as error:
                        report['cleanup_error'] = str(error)

                report['finished_at'] = (
                    datetime.now().astimezone().isoformat()
                )

                if output is not None:
                    with output.open(
                        'x',
                        encoding='utf-8',
                    ) as file:
                        json.dump(
                            report,
                            file,
                            ensure_ascii=False,
                            indent=2,
                            allow_nan=False,
                        )

                    print(f'实验记录：{output}')

            finally:
                node.destroy_node()

                if rclpy.ok():
                    rclpy.shutdown()

    return 0 if report['experiment_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())