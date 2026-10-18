from time import monotonic

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmClient(Node):
    def __init__(self):
        super().__init__(
            'arm_motion_client',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
        )

        self.positions = None

        self.joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.receive_state,
            10,
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

    def receive_state(self, message):
        positions = dict(zip(message.name, message.position))

        if all(name in positions for name in self.joints):
            self.positions = [positions[name] for name in self.joints]


def main():
    rclpy.init()
    node = ArmClient()

    try:
        deadline = monotonic() + 5.0

        while node.positions is None and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.positions is None:
            node.get_logger().error('5 秒内没有收到完整关节状态')
            return

        if not node.client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('未找到轨迹 Action，请检查接口名称和控制器')
            return

        start = node.positions.copy()
        target = start.copy()
        target[0] -= 0.15

        node.get_logger().info(f'起始位置：{start}')
        node.get_logger().info(f'目标位置：{target}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = node.joints

        start_point = JointTrajectoryPoint()
        start_point.positions = start
        start_point.velocities = [0.0] * 6
        start_point.time_from_start.sec = 1

        end_point = JointTrajectoryPoint()
        end_point.positions = target
        end_point.velocities = [0.0] * 6
        end_point.time_from_start.sec = 5

        goal.trajectory.points = [start_point, end_point]

        send_future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            node, send_future, timeout_sec=5.0
        )

        if not send_future.done():
            node.get_logger().error('目标响应超时，是否被接受尚不确定')
            return

        handle = send_future.result()

        if not handle.accepted:
            node.get_logger().error('控制器拒绝了轨迹')
            return

        node.get_logger().info('轨迹已接受，请观察机械臂底座关节')
        result_future = handle.get_result_async()

        rclpy.spin_until_future_complete(
            node, result_future, timeout_sec=20.0
        )

        if not result_future.done():
            node.get_logger().warning('等待结果超时，发送取消请求')
            cancel_future = handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                node, cancel_future, timeout_sec=5.0
            )
            rclpy.spin_until_future_complete(
                node, result_future, timeout_sec=5.0
            )

            if not result_future.done():
                node.get_logger().error('尚未确认任务停止，请检查仿真')
                return

        response = result_future.result()
        result = response.result

        node.get_logger().info(f'控制器结果码：{result.error_code}')

        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            node.get_logger().info('运动成功')
        elif response.status == GoalStatus.STATUS_CANCELED:
            node.get_logger().warning('运动已取消')
        else:
            node.get_logger().error(
                f'运动未成功：状态={response.status}，'
                f'说明={result.error_string}'
            )

    except KeyboardInterrupt:
        node.get_logger().warning('客户端被中断，不代表运动已取消')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()