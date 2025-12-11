from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the full Snydrone stack in simulation.

    NOTE: This is just the ROS 2 nodes. PX4 SITL and Isaac Sim / Pegasus
    are not started here yet; you'll hook those in later.
    """

    nodes = [
        Node(
            package="llm_agent",
            executable="llm_agent_node",
            name="llm_agent",
            output="screen",
        ),
        Node(
            package="shot_planner",
            executable="shot_planner_node",
            name="shot_planner",
            output="screen",
        ),
        Node(
            package="vision_tracker",
            executable="vision_tracker_node",
            name="vision_tracker",
            output="screen",
        ),
        Node(
            package="traj_follower",
            executable="traj_follower_node",
            name="traj_follower",
            output="screen",
        ),
        Node(
            package="dataset_logger",
            executable="dataset_logger_node",
            name="dataset_logger",
            output="screen",
        ),
    ]

    ld = LaunchDescription(nodes)

    # TODO: Add ExecuteProcess actions here to start PX4 SITL and Isaac Sim
    # when running on your Linux dev machine.

    return ld
