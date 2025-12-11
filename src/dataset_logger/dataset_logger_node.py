import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from snydrone_msgs.msg import TargetState


class DatasetLoggerNode(Node):
    """Stub dataset logger.

    Subscribes to a few topics and logs that it saw messages.
    In the future, this will be replaced with rosbag2-based recording and
    export to MP4/Parquet/JSONL.
    """

    def __init__(self) -> None:
        super().__init__("dataset_logger")
        self.img_sub = self.create_subscription(
            Image, "camera/image_raw", self.img_callback, 10
        )
        self.target_sub = self.create_subscription(
            TargetState, "target", self.target_callback, 10
        )
        self.img_count = 0
        self.target_count = 0
        self.get_logger().info("Dataset logger node started (stub).")

    def img_callback(self, msg: Image) -> None:
        self.img_count += 1
        if self.img_count % 30 == 0:
            self.get_logger().info(f"Received {self.img_count} images so far.")

    def target_callback(self, msg: TargetState) -> None:
        self.target_count += 1
        if self.target_count % 10 == 0:
            self.get_logger().info(
                f"Received {self.target_count} TargetState messages so far."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
