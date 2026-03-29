#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from anthropic import Anthropic
except ImportError:
    Anthropic = None

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("snydrone_llm_planner")

        self.sub_prompt = self.create_subscription(
            String,
            "/snydrone/shot/prompt",
            self.on_prompt,
            10
        )

        self.pub_spec = self.create_publisher(
            String,
            "/snydrone/shot/spec",
            10
        )

        # Initialize Anthropic client
        if Anthropic is None:
            self.get_logger().error("anthropic package not found. Please install: pip install anthropic")
            self.client = None
        else:
            api_key = os.environ.get("CLAUDE_API_KEY")
            if not api_key:
                self.get_logger().error("CLAUDE_API_KEY environment variable not set!")
                self.client = None
            else:
                self.client = Anthropic(api_key=api_key)

        self.system_prompt = """
        You are an autonomous cinematography drone controller.
        Your job is to translate natural language user requests into a strict JSON specification for a drone.
        The output must be pure JSON and parseable by json.loads(). Do not output any markdown formatting, backticks, or other text.
        
        The JSON must contain the following fields:
        "shot": A string, e.g. "orbit", "dolly_in", "dolly_out", "follow", "pan". Default to "orbit".
        "radius": A float representing radial distance to target in meters. Default to 3.0.
        "height": A float representing height above ground in meters. Default to 3.5.
        "speed": A float representing speed (angular for orbit, linear for dolly) in rad/s or m/s. Default 0.6.
        "duration_s": A float for how long the shot lasts in seconds. Default 10.0.
        "clockwise": A boolean, True or False. Default True.
        "look_at": A string, e.g. "target" or "none". Default "target".
        
        Example Output:
        {
            "shot": "orbit",
            "radius": 4.5,
            "height": 2.0,
            "speed": 0.5,
            "duration_s": 15.0,
            "clockwise": false,
            "look_at": "target"
        }
        """

        self.get_logger().info("LLMPlanner ready")
        self.get_logger().info("Listening: /snydrone/shot/prompt")
        self.get_logger().info("Publishing: /snydrone/shot/spec")

    def call_llm(self, prompt: str) -> str:
        if not self.client:
            self.get_logger().error("Anthropic client not initialized.")
            return None

        try:
            response = self.client.messages.create(
                model="claude-opus-4-6",
                system=self.system_prompt,
                messages=[
                    {"role": "user", "content": prompt}
                ],
                max_tokens=256,
                temperature=0.0
            )
            return response.content[0].text.strip()
        except Exception as e:
            self.get_logger().error(f"Anthropic API call failed: {e}")
            return None

    def on_prompt(self, msg: String):
        prompt = msg.data
        self.get_logger().info(f"Received PROMPT: {prompt}")

        llm_output = self.call_llm(prompt)
        
        if llm_output:
            self.get_logger().info(f"LLM Raw Output:\n{llm_output}")
            
            # Try to safely parse just to validate, but we'll publish the string
            try:
                # remove possible markdown backticks if the model ignored instructions
                if llm_output.startswith("```json"):
                    llm_output = llm_output[7:]
                if llm_output.startswith("```"):
                    llm_output = llm_output[3:]
                if llm_output.endswith("```"):
                    llm_output = llm_output[:-3]
                llm_output = llm_output.strip()

                spec = json.loads(llm_output)
                out = String()
                out.data = json.dumps(spec)
                self.pub_spec.publish(out)
                self.get_logger().info(f"Published SPEC: {out.data}")
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON from LLM: {e}")
        else:
            self.get_logger().error("Failed to get response from LLM or client not configured.")


def main():
    rclpy.init()
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
