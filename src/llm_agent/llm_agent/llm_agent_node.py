import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from snydrone_msgs.msg import CinematicShot


class LLMAgentNode(Node):
    """Dummy LLM agent.

    Subscribes to 'director_prompt' (std_msgs/String) and publishes a
    placeholder CinematicShot on 'shot_plan'.
    """

    def __init__(self) -> None:
        super().__init__("llm_agent")
        self.plan_pub = self.create_publisher(CinematicShot, "shot_plan", 10)
        self.prompt_sub = self.create_subscription(
            String, "director_prompt", self.prompt_callback, 10
        )
        self.get_logger().info("LLM Agent node started (dummy implementation).")

    def prompt_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received director prompt: '{msg.data}'")
        shot = CinematicShot()
        shot.header.stamp = self.get_clock().now().to_msg()
        shot.shot_type = "ORBIT"
        shot.target_id = 0
        shot.radius = 5.0
        shot.height_offset = 2.0
        shot.duration = 15.0
        shot.speed = 1.0
        shot.framing_x = 0.5
        shot.framing_y = 0.6
        shot.framing_scale = 0.4
        shot.params_json = "{}"
        self.plan_pub.publish(shot)
        self.get_logger().info("Published dummy CinematicShot for prompt.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LLMAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
