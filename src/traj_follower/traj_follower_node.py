from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class TrajFollowerNode(Node):
    """Dummy trajectory follower.

    Subscribes to 'trajectory' and logs basic info. This is where PX4 Offboard
    control (via MAVSDK or px4_ros_com) will be implemented.
    """

    def __init__(self) -> None:
        super().__init__("traj_follower")
        self.path_sub = self.create_subscription(
            Path, "trajectory", self.path_callback, 10
        )
        self.current_path: Optional[Path] = None
        self.get_logger().info("Trajectory follower node started (dummy implementation).")

    def path_callback(self, msg: Path) -> None:
        self.current_path = msg
        self.get_logger().info(
            f"Received trajectory with {len(msg.poses)} poses (no PX4 control yet)."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
