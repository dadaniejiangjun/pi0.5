from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest

from laptop.robot_client.supervised_j1_one_degree_trial import (
    execute as execute_j1_one_degree,
)
from laptop.robot_client.supervised_j1_step_trial import execute as execute_j1_step
from laptop.robot_client.supervised_j5_probe import execute as execute_j5
from laptop.robot_client.supervised_joint_replay import execute as execute_joint_replay
from laptop.robot_client.supervised_restore_gripper import execute as execute_gripper
from laptop.robot_client.supervised_restore_start import execute as execute_restore_start
from laptop.robot_client.supervised_smooth_joint_replay import (
    execute as execute_smooth_replay,
)


@pytest.mark.parametrize(
    "execute",
    [
        execute_j1_one_degree,
        execute_j1_step,
        execute_j5,
        execute_joint_replay,
        execute_gripper,
        execute_restore_start,
        execute_smooth_replay,
    ],
)
def test_legacy_raw_can_entrypoints_fail_before_adapter_access(execute):
    with pytest.raises(PermissionError, match="direct gs_usb CAN TX is disabled") as error:
        execute(operator_confirmed=True)
    event = error.value.audit_event
    assert event["schema"] == "piper_safety_bridge_hil_gate_v1"
    assert event["event_type"] == "hil_gate_rejection"
    assert event["hardware_control_allowed"] is False
    assert event["can_tx_attempted"] is False
    assert "hil_single_step_bridge_not_implemented" in event["reasons"]
    assert "live_hardware_gates_unverified" in event["reasons"]
    assert len(event["input_sha256"]) == 64
    serialized = str(error.value).rsplit("audit_event=", 1)[1]
    assert json.loads(serialized) == event


def test_every_direct_hil_send_is_behind_the_first_statement_guard():
    robot_client_dir = Path(__file__).resolve().parents[2] / "laptop" / "robot_client"
    guarded_modules = []

    class SendCallVisitor(ast.NodeVisitor):
        def __init__(self):
            self.function_stack = []
            self.send_call_stacks = []

        def visit_FunctionDef(self, node):
            self.function_stack.append(node.name)
            self.generic_visit(node)
            self.function_stack.pop()

        def visit_AsyncFunctionDef(self, node):
            self.function_stack.append(node.name)
            self.generic_visit(node)
            self.function_stack.pop()

        def visit_Call(self, node):
            if isinstance(node.func, ast.Attribute) and node.func.attr == "send":
                self.send_call_stacks.append(tuple(self.function_stack))
            self.generic_visit(node)

    for path in sorted(robot_client_dir.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        visitor = SendCallVisitor()
        visitor.visit(tree)
        if not visitor.send_call_stacks:
            continue

        execute = next(
            (node for node in tree.body
             if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
             and node.name == "execute"),
            None,
        )
        assert execute is not None, f"{path.name} has CAN send calls but no execute()"
        body = list(execute.body)
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            body.pop(0)
        assert body and isinstance(body[0], ast.Expr), (
            f"{path.name} must call the hard guard before other execute() work"
        )
        guard = body[0].value
        assert isinstance(guard, ast.Call) and isinstance(guard.func, ast.Name), (
            f"{path.name} must call reject_legacy_raw_can_tx() first"
        )
        assert guard.func.id == "reject_legacy_raw_can_tx", (
            f"{path.name} must call reject_legacy_raw_can_tx() first"
        )
        assert all("execute" in stack for stack in visitor.send_call_stacks), (
            f"{path.name} contains a CAN send outside execute()"
        )
        guarded_modules.append(path.name)

    assert guarded_modules, "expected to discover guarded HIL CAN transmit modules"
