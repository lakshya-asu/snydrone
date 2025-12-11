import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraStubNode(Node):
    """Camera stub.

    Publishes a constant gray RGB image and a simple CameraInfo at a fixed rate.
    This lets other nodes (vision_tracker, dataset_logger) run without a real sim.
    """

    def __init__(self) -> None:
        super().__init__("camera_stub")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("frame_id", "snydrone_camera")

        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        self.image_pub = self.create_publisher(
            Image, "snydrone/camera/color/image_raw", 10
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "snydrone/camera/camera_info", 10
        )

        period = 1.0 / self.fps if self.fps > 0.0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self._seq = 0
        self.get_logger().info(
            f"Camera stub started: {self.width}x{self.height} @ {self.fps} Hz "
            f"on frame_id='{self.frame_id}'"
        )

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        # Image
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height = self.height
        img.width = self.width
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = self.width * 3

        # Constant mid-gray image (not fancy, but enough for plumbing)
        gray_value = 128
        img.data = bytes([gray_value] * (self.width * self.height * 3))

        # CameraInfo
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height

        # Simple pinhole intrinsics (fake, but consistent)
        fx = fy = float(self.width)  # arbitrary
        cx = self.width / 2.0
        cy = self.height / 2.0
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]

        self.image_pub.publish(img)
        self.info_pub.publish(info)

        self._seq += 1
        if self._seq % int(self.fps) == 0:
            self.get_logger().info(f"Published {self._seq} frames.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
