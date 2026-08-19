"""Spec for talking to a local OpenAI-compatible model server.

The planner used to call a hosted API. It now calls a llama.cpp server
on the aircraft's own machine, which means no network, no key, and no
per-flight cost, and it also means a much smaller model whose output
has to be treated as hostile rather than trusted.

Everything here is the part that can be tested without a socket:
building the request, and getting a spec string back out of a reply
that may be malformed in any of the ways a local server actually
malforms it. The HTTP call itself is a dozen lines in the node.

Two settings below are not style choices and are asserted on purpose.
reasoning_effort must be "low": this build returns HTTP 500 for
"minimal", and the default is high enough to burn the whole token
budget on reasoning and return an empty string. max_tokens must leave
room for that reasoning: a refusal-worthy prompt ("do a barrel roll")
measured 413 completion tokens against about 190 for a normal one, so
a budget sized for the normal case truncates exactly the inputs most
likely to be dangerous.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_brain.planner_client import (  # noqa: E402
    PlannerError,
    build_request,
    extract_spec_text,
)


# ------------------------------------------------------------ request

def test_request_carries_the_system_prompt_and_the_user_words():
    req = build_request("SYSTEM TEXT", "orbit the tower")
    assert req["messages"][0] == {"role": "system", "content": "SYSTEM TEXT"}
    assert req["messages"][1] == {"role": "user", "content": "orbit the tower"}


def test_reasoning_effort_is_low():
    # "minimal" is a 500 on this build; the default spends the budget
    # thinking and returns nothing.
    assert build_request("s", "p")["reasoning_effort"] == "low"


def test_temperature_is_zero():
    # The same spoken order must produce the same flight, every time.
    assert build_request("s", "p")["temperature"] == 0.0


def test_token_budget_has_room_for_reasoning():
    assert build_request("s", "p")["max_tokens"] >= 600


def test_token_budget_is_overridable():
    assert build_request("s", "p", max_tokens=1500)["max_tokens"] == 1500


@pytest.mark.parametrize("bad", ["", "   ", None])
def test_an_empty_prompt_is_refused(bad):
    with pytest.raises(PlannerError):
        build_request("s", bad)


# ------------------------------------------------------------- replies

def reply(content, finish="stop", reasoning=None):
    msg = {"role": "assistant", "content": content}
    if reasoning is not None:
        msg["reasoning_content"] = reasoning
    return {"choices": [{"message": msg, "finish_reason": finish}]}


def test_the_spec_text_comes_back():
    assert extract_spec_text(reply('{"shot": "orbit"}')) == '{"shot": "orbit"}'


def test_surrounding_whitespace_is_dropped():
    assert extract_spec_text(reply('  {"shot": "orbit"}\n\n')) == \
        '{"shot": "orbit"}'


def test_reasoning_is_not_part_of_the_spec():
    # The server returns reasoning in its own field. It must never be
    # concatenated onto the answer.
    out = extract_spec_text(
        reply('{"shot": "orbit"}', reasoning="The user wants a circle."))
    assert "circle" not in out


def test_a_truncated_reply_is_an_error_not_a_half_spec():
    # finish_reason "length" means the budget ran out mid-JSON. Parsing
    # that would be parsing a fragment.
    with pytest.raises(PlannerError):
        extract_spec_text(reply('{"shot": "orb', finish="length"))


def test_an_empty_reply_is_an_error():
    # What the default reasoning_effort produces: all budget spent
    # thinking, nothing said.
    for content in ("", "   ", None):
        with pytest.raises(PlannerError):
            extract_spec_text(reply(content))


def test_a_reply_with_no_choices_is_an_error():
    with pytest.raises(PlannerError):
        extract_spec_text({"choices": []})


def test_a_reply_missing_the_message_is_an_error():
    with pytest.raises(PlannerError):
        extract_spec_text({"choices": [{"finish_reason": "stop"}]})


def test_a_server_error_payload_is_an_error():
    with pytest.raises(PlannerError):
        extract_spec_text({"error": {"message": "context overflow"}})


def test_the_error_says_what_came_back():
    # A silent failure on an aircraft is the worst kind. The message
    # has to be enough to diagnose from a flight log alone.
    with pytest.raises(PlannerError) as e:
        extract_spec_text(reply('{"shot": "orb', finish="length"))
    assert "length" in str(e.value).lower()


def test_extraction_does_not_validate_the_spec():
    # Validation is shot_spec.py's job and it is tested there. This
    # layer must hand back whatever the model said, including nonsense,
    # so that exactly one component decides what is flyable.
    assert extract_spec_text(reply("not json at all")) == "not json at all"
