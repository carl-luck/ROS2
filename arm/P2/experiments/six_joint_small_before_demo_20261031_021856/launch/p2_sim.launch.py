from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "runtime_config_package": "ur5e_mpc_control",
            "controllers_file": "p2_controllers.yaml",
            "initial_joint_controller": "forward_velocity_controller",
            "start_joint_controller": "true",
            "gazebo_gui": "true",
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription([simulation])