#!/usr/bin/env python3

import json
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def rule_based_plan(prompt: str) -> dict:
    """
    Minimal planner:
    converts natural language to a structured shot spec (JSON-able dict)

    We keep it dumb on purpose for now.
    Later you swap this function with an LLM call.
    """

    def first_float(text: str):
        # grabs full numbers like "4", "3.2", "12.0"
        m = re.search(r"([0-9]+(?:\.[0-9]+)?)", text)
        return float(m.group(1)) if m else None

    p = prompt.lower().strip()

    # defaults
    spec = {
        "shot": "orbit",
        "radius": 3.0,
        "height": 3.5,
        "speed": 0.6,
        "look_at": "target",
        "clockwise": True,
        "duration_s": 10.0
    }

    # shot type
    if "orbit" in p or "circle" in p:
        spec["shot"] = "orbit"
    elif "follow" in p or "chase" in p:
        spec["shot"] = "follow"
    elif "dolly" in p or "push in" in p or "move closer" in p:
        spec["shot"] = "dolly_in"
    elif "pull out" in p or "move away" in p:
        spec["shot"] = "dolly_out"

    # direction
    if "counter" in p or "anticlock" in p or "ccw" in p:
        spec["clockwise"] = False
    if "clockwise" in p or "cw" in p:
        spec["clockwise"] = True

    # radius
    m = re.search(r"(radius\s*=?\s*[0-9]+(?:\.[0-9]+)?)|([0-9]+(?:\.[0-9]+)?\s*m\s*radius)", p)
    if m:
        val = first_float(m.group(0))
        if val is not None:
            spec["radius"] = val

    # height
    m = re.search(r"(height\s*=?\s*[0-9]+(?:\.[0-9]+)?)|([0-9]+(?:\.[0-9]+)?\s*m\s*high)", p)
    if m:
        val = first_float(m.group(0))
        if val is not None:
            spec["height"] = val

    # speed
    m = re.search(r"(speed\s*=?\s*[0-9]+(?:\.[0-9]+)?)", p)
    if m:
        val = first_float(m.group(0))
        if val is not None:
            spec["speed"] = val

    # duration
    m = re.search(r"(for\s*[0-9]+(?:\.[0-9]+)?\s*s)", p)
    if m:
        val = first_float(m.group(0))
        if val is not None:
            spec["duration_s"] = val

    if "don't look" in p or "no lookat" in p:
        spec["look_at"] = "none"

    return spec



class ShotPlannerNode(Node):
    def __init__(self):
        super().__init__("snydrone_shot_planner")

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

        self.get_logger().info("ShotPlanner ready")
        self.get_logger().info("Listening: /snydrone/shot/prompt")
        self.get_logger().info("Publishing: /snydrone/shot/spec")

    def on_prompt(self, msg: String):
        prompt = msg.data
        spec = rule_based_plan(prompt)

        out = String()
        out.data = json.dumps(spec)
        self.pub_spec.publish(out)

        self.get_logger().info(f"PROMPT: {prompt}")
        self.get_logger().info(f"SPEC: {out.data}")


def main():
    rclpy.init()
    node = ShotPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
