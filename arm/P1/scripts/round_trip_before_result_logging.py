import argparse
import csv
import math
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from time import monotonic

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RoundTrip(Node):
    def __init__(self, end_time):
        super().__init__(
            'arm_round_trip',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.end_time = end_time

        self.joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.positions = None
        self.state_stamp = None
        self.description = None
        self.goal_handle = None
        self.result_future = None
        self.send_future = None

        self.phase = ''
        self.samples = 0
        self.last_print = -math.inf
        self.bad_feedback = False

        self.file = None
        self.writer = None

        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.receive_state,
            10,
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.description_sub = self.create_subscription(
            String,
            '/robot_description',
            self.receive_description,
            qos,
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

    def receive_state(self, message):
        values = dict(zip(message.name, message.position))

        if all(name in values for name in self.joints):
            positions = [values[name] for name in self.joints]

            if all(math.isfinite(value) for value in positions):
                self.positions = positions
                self.state_stamp = seconds(message.header.stamp)

    def receive_description(self, message):
        self.description = message.data

    def wait_for(self, condition, timeout):
        deadline = monotonic() + timeout

        while rclpy.ok() and not condition():
            remaining = deadline - monotonic()

            if remaining <= 0:
                return False

            rclpy.spin_once(
                self,
                timeout_sec=min(0.1, remaining),
            )

        return rclpy.ok() and condition()

    def fresh_state(self):
        previous = self.state_stamp

        ready = self.wait_for(
            lambda: self.positions is not None
            and self.state_stamp != previous
            and self.get_clock().now().nanoseconds > 0,
            5.0,
        )

        if not ready:
            raise RuntimeError(
                '5 秒内未收到推进的仿真关节状态；检查是否暂停。'
            )

        return self.positions.copy()

    def read_limits(self):
        if not self.wait_for(
            lambda: self.description is not None,
            5.0,
        ):
            raise RuntimeError(
                '未收到 /robot_description，未发送运动目标。'
            )

        robot = ET.fromstring(self.description)
        limits = {}

        for joint in robot.findall('joint'):
            name = joint.get('name')

            if name not in self.joints:
                continue

            if joint.get('type') == 'continuous':
                limits[name] = (-math.inf, math.inf)
            else:
                limit = joint.find('limit')

                if limit is None:
                    raise RuntimeError(f'{name} 缺少限位。')

                limits[name] = (
                    float(limit.get('lower')),
                    float(limit.get('upper')),
                )

        if any(name not in limits for name in self.joints):
            raise RuntimeError('模型中的关节名称或限位不完整。')

        return limits

    def feedback(self, message):
        data = message.feedback
        count = len(data.joint_names)

        arrays = [
            data.desired.positions,
            data.actual.positions,
            data.error.positions,
        ]

        if any(len(values) != count for values in arrays):
            self.bad_feedback = True
            return

        indexes = {
            name: i
            for i, name in enumerate(data.joint_names)
        }

        if any(name not in indexes for name in self.joints):
            self.bad_feedback = True
            return

        if not all(
            math.isfinite(value)
            for values in arrays
            for value in values
        ):
            self.bad_feedback = True
            return

        sim_time = seconds(data.header.stamp)
        trajectory_time = seconds(data.desired.time_from_start)

        for name in self.joints:
            i = indexes[name]

            desired, actual, error = [
                values[i] for values in arrays
            ]

            self.writer.writerow([
                self.phase,
                sim_time,
                trajectory_time,
                name,
                desired,
                actual,
                error,
                desired - actual,
            ])

        self.samples += 1

        if monotonic() - self.last_print >= 0.5:
            i = indexes[self.joints[0]]

            print(
                f'[{self.phase}] sim={sim_time:.3f}s '
                f'desired={data.desired.positions[i]:.4f} '
                f'actual={data.actual.positions[i]:.4f} '
                f'error={data.error.positions[i]:+.6f} rad',
                flush=True,
            )

            self.file.flush()
            self.last_print = monotonic()

    def cancel_pending(self):
        if self.send_future is not None and self.goal_handle is None:
            self.wait_for(self.send_future.done, 5.0)

            if not self.send_future.done():
                print(
                    '目标响应仍未知；无法确认是否在执行，请检查仿真。'
                )
                return

            self.goal_handle = self.send_future.result()

        if self.goal_handle is None or not self.goal_handle.accepted:
            return

        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()

        if self.result_future.done():
            return

        print('发送取消请求，等待服务端最终状态……')

        future = self.goal_handle.cancel_goal_async()
        self.wait_for(future.done, 3.0)

        if self.wait_for(self.result_future.done, 5.0):
            response = self.result_future.result()

            print(
                f'最终 Action 状态={response.status}，'
                f'结果码={response.result.error_code}'
            )
        else:
            print(
                '尚未确认任务停止；若 Gazebo 暂停，'
                '请恢复后检查控制器状态。'
            )

    def move(self, phase, target):
        start = self.fresh_state()

        if max(
            abs(a - b)
            for a, b in zip(start, target)
        ) > 0.35:
            raise RuntimeError(
                '单段位置变化超过本实验的 0.35 rad 范围，停止发送。'
            )

        self.phase = phase
        self.samples = 0
        self.bad_feedback = False
        self.last_print = -math.inf
        self.goal_handle = None
        self.result_future = None

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        for positions, duration in [
            (start, 1),
            (target, self.end_time),
        ]:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6
            point.time_from_start.sec = duration

            goal.trajectory.points.append(point)

        print(
            f'\n[{phase}] 底座 '
            f'{start[0]:.4f} → {target[0]:.4f} rad'
        )

        wall_start = monotonic()

        self.send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback,
        )

        if not self.wait_for(self.send_future.done, 5.0):
            raise RuntimeError(
                '目标响应超时，是否接受尚不确定；停止后续运动。'
            )

        self.goal_handle = self.send_future.result()

        if not self.goal_handle.accepted:
            raise RuntimeError(f'{phase} 目标被拒绝。')

        self.result_future = self.goal_handle.get_result_async()
        sim_start = self.get_clock().now().nanoseconds * 1e-9

        wait_timeout = max(20.0, 3.0 * self.end_time)

        if not self.wait_for(
            self.result_future.done,
            wait_timeout,
        ):
            sim_elapsed = (
                self.get_clock().now().nanoseconds * 1e-9
                - sim_start
            )

            raise RuntimeError(
                f'等待结果超过 {wait_timeout:.1f} 秒墙钟时间，'
                f'期间仿真推进 {sim_elapsed:.3f}s；'
                '请检查暂停或仿真速度，'
                '不能仅据此判定控制故障。'
            )

        response = self.result_future.result()

        print(
            f'[{phase}] status={response.status}, '
            f'code={response.result.error_code}, '
            f'墙钟耗时={monotonic() - wall_start:.2f}s, '
            f'反馈帧数={self.samples}'
        )

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            raise RuntimeError(
                f'运动未成功：{response.result.error_string}'
            )

        if self.samples == 0 or self.bad_feedback:
            raise RuntimeError(
                '运动返回成功，但反馈缺失或字段异常，记录实验未通过。'
            )

        actual = self.fresh_state()

        deviation = max(
            abs(a - b)
            for a, b in zip(actual, target)
        )

        print(
            f'[{phase}] 结束后底座={actual[0]:.6f} rad，'
            f'六关节最大终点偏差={deviation:.6f} rad'
        )

        if deviation > 0.01:
            raise RuntimeError(
                '终点偏差超过本实验检查阈值 0.01 rad，停止后续运动。'
            )


def main():
    parser = argparse.ArgumentParser(
        description='UR5e 固定位置往返与反馈记录'
    )

    parser.add_argument(
        '--duration',
        type=int,
        choices=[5, 9],
        default=5,
        help='每段总仿真时间：5 秒或 9 秒，包含约 1 秒保持',
    )

    args = parser.parse_args()

    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = RoundTrip(args.duration)
    passed = False

    print(
        f'本次每段总时间：{args.duration} s，'
        f'其中移动约 {args.duration - 1} s（仿真时间）'
    )

    try:
        current = node.fresh_state()
        limits = node.read_limits()
        targets = []

        for angle in [0.15, 0.30, 0.15]:
            target = current.copy()
            target[0] = angle

            for name, value in zip(node.joints, target):
                lower, upper = limits[name]

                if not lower <= value <= upper:
                    raise RuntimeError(
                        f'{name} 目标 {value} '
                        f'超出模型限位 [{lower}, {upper}]。'
                    )

            targets.append(target)

        print(
            f'模型中的底座限位：'
            f'{limits[node.joints[0]]} rad'
        )

        if not node.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('未找到轨迹 Action 服务端。')

        folder = Path(__file__).resolve().parent.parent / 'data'
        folder.mkdir(parents=True, exist_ok=True)

        path = folder / (
            f'round_trip_T{args.duration}s_'
            f'{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
        )

        node.file = path.open(
            'x',
            newline='',
            encoding='utf-8',
        )

        node.writer = csv.writer(node.file)

        node.writer.writerow([
            'phase',
            'sim_time_s',
            'desired_time_from_start_s',
            'joint',
            'desired_rad',
            'actual_rad',
            'controller_error_rad',
            'desired_minus_actual_rad',
        ])

        print(f'CSV：{path}')

        for phase, target in zip(
            ['A_start', 'B', 'A_return'],
            targets,
        ):
            node.move(phase, target)

        passed = True

        print(
            '\n本次往返和反馈记录检查通过。'
            'CSV 尚未进行 RMSE 分析。'
        )

    except KeyboardInterrupt:
        print('\n客户端收到 Ctrl+C，尝试取消未完成目标。')

    except Exception as error:
        print(f'\n实验未通过：{error}')

    finally:
        try:
            node.cancel_pending()
        finally:
            if node.file is not None:
                node.file.close()

            node.destroy_node()
            rclpy.shutdown()

    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())