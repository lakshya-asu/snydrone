"""Planner client: request building and reply extraction for a local llama.cpp server."""


class PlannerError(Exception):
    """Raised when the planner request or reply is invalid."""


DEFAULT_MAX_TOKENS = 900


def build_request(system_prompt, user_prompt, max_tokens=DEFAULT_MAX_TOKENS):
    """Build the JSON body dict for the local model server.

    Args:
        system_prompt: The system instruction string, passed through unchanged.
        user_prompt: The operator's input string.
        max_tokens: Completion token budget.

    Returns:
        A dict suitable as the JSON request body.

    Raises:
        PlannerError: If user_prompt is not a string, is empty, or is only whitespace.
    """
    if not isinstance(user_prompt, str):
        raise PlannerError(
            "user_prompt must be a string, got {}".format(type(user_prompt).__name__)
        )
    if user_prompt == "" or user_prompt.strip() == "":
        raise PlannerError("user_prompt must not be empty or whitespace-only")

    return {
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ],
        "max_tokens": max_tokens,
        "temperature": 0.0,
        "reasoning_effort": "low",
    }


def extract_spec_text(payload):
    """Extract the model's spec text from a parsed JSON reply.

    Args:
        payload: The parsed JSON reply dict from the server.

    Returns:
        The stripped spec text string.

    Raises:
        PlannerError: If the payload is malformed or indicates an error.
    """
    if not isinstance(payload, dict):
        raise PlannerError(
            "Reply payload is not a dict, got {}".format(type(payload).__name__)
        )

    if payload.get("error"):
        raise PlannerError(
            "Server returned an error: {}".format(payload["error"])
        )

    choices = payload.get("choices")
    if not choices:
        raise PlannerError(
            "Reply has no choices; keys present: {}".format(
                sorted(k for k in payload.keys() if k is not None)
            )
        )

    first_choice = choices[0]
    if not isinstance(first_choice, dict):
        raise PlannerError(
            "First choice is not a dict, got {}".format(type(first_choice).__name__)
        )

    message = first_choice.get("message")
    if not isinstance(message, dict):
        raise PlannerError(
            "First choice has no message dict; choice keys: {}".format(
                sorted(first_choice.keys()) if isinstance(first_choice, dict) else "n/a"
            )
        )

    finish_reason = first_choice.get("finish_reason")

    if finish_reason == "length":
        raise PlannerError(
            "Model output was truncated: finish_reason is 'length'. "
            "The response was cut off at max_tokens and the content is a fragment. "
            "Increase max_tokens or shorten the prompt."
        )

    content = message.get("content")
    if not isinstance(content, str) or content.strip() == "":
        raise PlannerError(
            "Message content is missing, not a string, or blank. "
            "finish_reason: {}".format(finish_reason)
        )

    return content.strip()
