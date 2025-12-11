import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image
from snydrone_msgs.msg import TargetState


class VisionTrackerNode(Node):
    """Dummy target tracker.

    Publishes a fake TargetState on 'target' at 10 Hz, simulating a subject
    moving in a small circle.
    """

    def __init__(self) -> None:
        super().__init__("vision_tracker")
        self.declare_parameter("publish_rate_hz", 10.0)
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.target_pub = self.create_publisher(TargetState, "target", 10)
        self.img_count = 0
        self.img_sub = self.create_subscription(
            Image,
            "snydrone/camera/color/image_raw",
            self.img_callback,
            10,
        )

        self.t = 0.0
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info("Vision tracker node started (dummy implementation).")

    def _on_timer(self) -> None:
        self.t += 0.1
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_id = 0
        msg.position = Point()
        msg.position.x = 0.0 + 1.5 * math.cos(self.t)
        msg.position.y = 0.0 + 1.5 * math.sin(self.t)
        msg.position.z = 1.5
        msg.velocity = Vector3()
        msg.velocity.x = -1.5 * math.sin(self.t)
        msg.velocity.y = 1.5 * math.cos(self.t)
        msg.velocity.z = 0.0
        msg.distance = 3.0
        msg.frame_error_x = 0.0
        msg.frame_error_y = 0.0
        self.target_pub.publish(msg)

        def img_callback(self, msg: Image) -> None:
        # For now we just count frames; real vision will come later.
            self.img_count += 1
            if self.img_count % 30 == 0:
                self.get_logger().info(f"Vision tracker saw {self.img_count} images.")



def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
