#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class ImageRelayNode(Node):
    def __init__(self):
        super().__init__("snydrone_image_relay")

        # Input topics from Isaac Sim
        self.input_rgb_topic = "/rgb"
        self.input_caminfo_topic = "/camera_info"

        # Output topics for SnyDrone
        self.output_rgb_topic = "/snydrone/camera/image_raw"
        self.output_caminfo_topic = "/snydrone/camera/camera_info"

        # QoS: keep small queue, reliable is fine for sim
        self.rgb_pub = self.create_publisher(Image, self.output_rgb_topic, 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, self.output_caminfo_topic, 10)

        self.rgb_sub = self.create_subscription(
            Image, self.input_rgb_topic, self.on_rgb, 10
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, self.input_caminfo_topic, self.on_caminfo, 10
        )

        self.get_logger().info("SnyDrone Image Relay started")
        self.get_logger().info(f"Relaying {self.input_rgb_topic} -> {self.output_rgb_topic}")
        self.get_logger().info(f"Relaying {self.input_caminfo_topic} -> {self.output_caminfo_topic}")

    def on_rgb(self, msg: Image):
        # Preserve header timestamps and frame_id
        self.rgb_pub.publish(msg)

    def on_caminfo(self, msg: CameraInfo):
        self.caminfo_pub.publish(msg)


def main():
    rclpy.init()
    node = ImageRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
