"""PiPER contract-only policy adapters.

These modules describe the semantic boundary for a future custom PiPER policy.
They intentionally do not load a checkpoint, talk to CAN, or issue robot
commands.
"""

from .piper_inputs import PiperInputs
from .piper_outputs import PiperOutputs, action_pad_mask, pad_action_7_to_model, unpad_model_action_to_7

__all__ = [
    "PiperInputs",
    "PiperOutputs",
    "action_pad_mask",
    "pad_action_7_to_model",
    "unpad_model_action_to_7",
]
