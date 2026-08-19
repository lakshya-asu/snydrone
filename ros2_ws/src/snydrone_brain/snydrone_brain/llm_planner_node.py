#!/usr/bin/env python3
"""Turn what the operator said into a shot specification the aircraft
will accept, using a language model running on this machine.

Inference is local. The node talks to a llama.cpp server over its
OpenAI-compatible endpoint, so there is no network dependency, no API
key, and no per-flight cost, and the aircraft keeps working somewhere
with no signal. Measured on Qwen3.8-27B at reasoning_effort low, a shot
command comes back in four to five seconds.

The trade is that a local model of this size is a good deal looser than
a hosted frontier model, so nothing it says is trusted. Every reply goes
through shot_spec.parse_shot_spec before it is published, which is the
same validation the browser demo and the unit tests exercise, and the
only thing that reaches /snydrone/shot/spec is a spec that survived it.

One limit is worth stating plainly, because it is not obvious and the
validator cannot fix it. Asked for something the vocabulary has no word
for, the model does not necessarily say so. Measured: "do a barrel roll"
came back as a valid orbit at radius 1.5 and speed 2.0, which passes
validation because it is in range and in the enum. Validation catches
malformed and out-of-range output. It cannot catch a plausible
substitution. Treat the published spec as what the model understood,
not as confirmation that it understood correctly.
"""

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from snydrone_brain.planner_client import (
    PlannerError,
    build_request,
    extract_spec_text,
)
from snydrone_brain.shot_spec import ShotSpecError, parse_shot_spec

DEFAULT_ENDPOINT = "http://127.0.0.1:8080/v1/chat/completions"

# Long enough for the slowest measured reply (8.5 s, on a prompt the
# model had to think hard about) with headroom for a cold slot, short
# enough that a wedged server is obvious rather than a hang.
DEFAULT_TIMEOUT_S = 30.0


def _refusal_reason(raw):
    """The reason, if the model declined; None if it answered.

    Deliberately narrow. Only a well-formed object whose sole meaningful
    key is "refuse" counts, so a spec that happens to mention the word
    still gets validated normally rather than being silently dropped.
    """
    try:
        obj = json.loads(raw)
    except (ValueError, TypeError):
        return None                  # not JSON: shot_spec will handle it
    if not isinstance(obj, dict) or "refuse" not in obj:
        return None
    if any(k in obj for k in ("shot", "radius", "height", "duration_s")):
        return None                  # it answered and editorialised
    reason = obj.get("refuse")
    return str(reason).strip() or "no reason given"


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("snydrone_llm_planner")

        self.declare_parameter("endpoint", DEFAULT_ENDPOINT)
        self.declare_parameter("timeout_s", DEFAULT_TIMEOUT_S)
        self.declare_parameter("max_tokens", 900)

        self.endpoint = self.get_parameter("endpoint").value
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.max_tokens = int(self.get_parameter("max_tokens").value)

        self.sub_prompt = self.create_subscription(
            String, "/snydrone/shot/prompt", self.on_prompt, 10)
        self.pub_spec = self.create_publisher(
            String, "/snydrone/shot/spec", 10)

        # The limits are deliberately NOT stated here. Telling the model
        # the ranges makes it clip its own answers, which sounds good and
        # is worse: the operator asks for 400 m, the model quietly says
        # 20, and the log reads clean because the validator had nothing
        # left to clamp. Enforcement belongs in one place, after the
        # model, where it is visible. Measured, not assumed: with the
        # ranges in this prompt, "orbit at four hundred meters" produced
        # radius 20 and no warning at all.
        self.system_prompt = """You are an autonomous cinematography drone controller.
Translate the operator's request into a strict JSON specification.
Output pure JSON only. No prose, no markdown, no backticks.

Fields, all required:
"shot": one of "orbit", "dolly_in", "dolly_out", "follow", "pan".
"radius": float, metres from the target.
"height": float, metres above ground.
"speed": float, metres per second along the flight path, for every shot type.
"duration_s": float, seconds.
"clockwise": boolean.
"look_at": "target" or "none".

Report the values the operator actually asked for. Do not clip them to
what seems reasonable; something downstream checks the ranges and needs
to see the real request.

If the request is not one of those five camera moves, or is not a
flight instruction at all, do not invent a substitute. Output exactly:
{"refuse": "<short reason>"}

Example:
{"shot": "orbit", "radius": 4.5, "height": 2.0, "speed": 0.5, "duration_s": 15.0, "clockwise": false, "look_at": "target"}
Refusal example:
{"refuse": "a barrel roll is not one of the supported camera moves"}
"""

        self.get_logger().info(f"LLMPlanner ready, local inference at {self.endpoint}")
        self.get_logger().info("Listening: /snydrone/shot/prompt")
        self.get_logger().info("Publishing: /snydrone/shot/spec")

    def call_llm(self, prompt: str) -> str:
        """Ask the local server. Raises PlannerError on any failure."""
        body = build_request(self.system_prompt, prompt,
                             max_tokens=self.max_tokens)
        req = urllib.request.Request(
            self.endpoint,
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read())
        except urllib.error.URLError as e:
            # By far the most likely failure in the field, and the one
            # worth naming precisely: the model server is not running.
            raise PlannerError(
                f"no answer from the local model at {self.endpoint}: {e}. "
                f"Is llama-server up?") from e
        except json.JSONDecodeError as e:
            raise PlannerError(f"model server sent malformed JSON: {e}") from e

        return extract_spec_text(payload)

    def on_prompt(self, msg: String):
        prompt = msg.data
        self.get_logger().info(f"PROMPT: {prompt}")

        try:
            raw = self.call_llm(prompt)
        except PlannerError as e:
            self.get_logger().error(f"planner failed, nothing published: {e}")
            return

        self.get_logger().info(f"model said: {raw}")

        # A refusal is a successful outcome, not an error. It is the only
        # defence against the failure mode below: asked for something
        # outside the vocabulary, a model that must answer will produce a
        # valid-looking shot, and no validator downstream can tell that
        # from a real one. Giving it a way to say no is what makes
        # "make me a sandwich" stop being a flight.
        refusal = _refusal_reason(raw)
        if refusal is not None:
            self.get_logger().warn(f"REFUSED by planner, nothing published: {refusal}")
            return

        try:
            spec = parse_shot_spec(raw)
        except ShotSpecError as e:
            # The whole point of the node. A spec that does not validate
            # never reaches the executor, and the aircraft does nothing,
            # which is the correct response to an order nobody can read.
            self.get_logger().error(f"REJECTED, not published: {e}")
            return

        clamped = spec.pop("clamped", [])
        if clamped:
            self.get_logger().warn(
                f"clamped to limits: {', '.join(clamped)}. "
                f"The aircraft will fly the clamped values, not what was asked.")

        out = String()
        out.data = json.dumps(spec)
        self.pub_spec.publish(out)
        self.get_logger().info(f"PUBLISHED: {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
