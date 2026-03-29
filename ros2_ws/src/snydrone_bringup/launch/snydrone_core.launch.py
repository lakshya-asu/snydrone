import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The LLM planner that listens to /snydrone/shot/prompt
        Node(
            package='snydrone_brain',
            executable='llm_planner_node',
            name='snydrone_llm_planner',
            output='screen',
        ),
        # The shot executor (generates the motion setpoint based on pure JSON)
        Node(
            package='snydrone_brain',
            executable='shot_executor_node',
            name='snydrone_shot_executor',
            output='screen',
            parameters=[{'rate_hz': 20.0}]
        ),
        # The PX4 Offboard Adapter (relays poses to PX4 microdds)
        Node(
            package='snydrone_px4',
            executable='px4_offboard_adapter_node',
            name='snydrone_px4_offboard_adapter',
            output='screen',
            parameters=[
                {'pose_frame': 'ENU'},
                {'rate_hz': 20.0},
                {'auto_arm_and_offboard': True}
            ]
        ),
    ])
