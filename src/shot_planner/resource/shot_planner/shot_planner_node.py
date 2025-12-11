import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from snydrone_msgs.msg import CinematicShot, TargetState


class ShotPlannerNode(Node):
    """Simple shot planner.

    Listens for CinematicShot on 'shot_plan' and publishes a dummy circular
    trajectory on 'trajectory' (nav_msgs/Path).
    """

    def __init__(self) -> None:
        super().__init__("shot_planner")
        self.declare_parameter("path_frame_id", "map")
        self.path_frame_id = (
            self.get_parameter("path_frame_id").get_parameter_value().string_value
        )
        self.current_target: Optional[TargetState] = None

        self.shot_sub = self.create_subscription(
            CinematicShot, "shot_plan", self.shot_callback, 10
        )
        self.target_sub = self.create_subscription(
            TargetState, "target", self.target_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "trajectory", 10)

        self.get_logger().info("Shot planner node started (dummy implementation).")

    def target_callback(self, msg: TargetState) -> None:
        self.current_target = msg

    def shot_callback(self, shot: CinematicShot) -> None:
        self.get_logger().info(f"Received CinematicShot of type {shot.shot_type}")
        path = self._build_orbit_path(shot)
        self.path_pub.publish(path)
        self.get_logger().info(
            f"Published trajectory with {len(path.poses)} poses (dummy orbit)."
        )

    def _build_orbit_path(self, shot: CinematicShot) -> Path:
        # Simple flat circle around (0,0) or around target if known.
        center_x = 0.0
        center_y = 0.0
        if self.current_target is not None:
            center_x = self.current_target.position.x
            center_y = self.current_target.position.y

        radius = shot.radius if shot.radius > 0.0 else 5.0
        height = (
            (self.current_target.position.z if self.current_target else 0.0)
            + shot.height_offset
        )

        num_steps = 60

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.path_frame_id

        for i in range(num_steps + 1):
            angle = 2.0 * math.pi * (i / num_steps)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = center_x + radius * math.cos(angle)
            pose.pose.position.y = center_y + radius * math.sin(angle)
            pose.pose.position.z = height
            pose.pose.orientation.w = 1.0  # yaw not set yet
            path.poses.append(pose)

        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShotPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
