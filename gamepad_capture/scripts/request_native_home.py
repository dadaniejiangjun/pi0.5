"""Request bounded zero return from the already running native CAN owner."""
from __future__ import annotations

import argparse
import json
import sys
import uuid
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from pi05_piper_interfaces.srv import HomeArm
from std_srvs.srv import SetBool


def _home_pause(node, paused, *, required=False):
    client = node.create_client(SetBool, '/gamepad/control/home_pause')
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            executor_present = any(name in ('gamepad_execution', 'quest_execution')
                                   for name in node.get_node_names())
            if required or executor_present:
                raise RuntimeError('home pause barrier unavailable; restart the updated gamepad executor before homing')
            return False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            request = SetBool.Request()
            request.data = paused
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if not future.done() or future.result() is None:
                raise RuntimeError('home pause barrier response unknown; no home request sent or control resumed')
            result = future.result()
            if result.success:
                return True
            if result.message != 'WAIT_PENDING_RPC':
                raise RuntimeError(f'home pause barrier refused: {result.message}')
            rclpy.spin_once(node, timeout_sec=0.05)
        raise RuntimeError('home pause barrier did not drain; home blocked')
    finally:
        node.destroy_client(client)


def _parameter_scope(node, node_name: str, timeout_s: float) -> str:
    client = node.create_client(GetParameters, f'{node_name}/get_parameters')
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f'parameter service unavailable for {node_name}')
    request = GetParameters.Request()
    request.names = ['task_scope_id']
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        raise RuntimeError('timed out reading the active task scope')
    values = list(future.result().values)
    if len(values) != 1 or values[0].type != Parameter.Type.STRING.value:
        raise RuntimeError('active native control node did not expose a string task_scope_id')
    scope = str(values[0].string_value)
    if not scope:
        raise RuntimeError('active native control node has no task_scope_id')
    return scope


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-scope-id', default='')
    parser.add_argument('--request-id', default='')
    parser.add_argument('--node-name', default='/pi05_native_can_state')
    parser.add_argument('--timeout-s', type=float, default=100.0)
    args = parser.parse_args(argv)
    if not 1.0 <= args.timeout_s <= 110.0:
        parser.error('--timeout-s must be within 1..110 seconds')

    rclpy.init()
    node = rclpy.create_node('pi05_request_native_home')
    try:
        scope = args.task_scope_id.strip() or _parameter_scope(
            node, args.node_name.rstrip('/'), min(args.timeout_s, 10.0))
        client = node.create_client(HomeArm, '/piper/control/home_arm')
        if not client.wait_for_service(timeout_sec=min(args.timeout_s, 10.0)):
            raise RuntimeError('native home service is unavailable; start the existing control owner first')
        request = HomeArm.Request()
        request.request_id = args.request_id.strip() or f'auto-home-{uuid.uuid4().hex}'
        request.confirmation_token = scope
        paused = _home_pause(node, True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_s)
        if not future.done():
            raise RuntimeError('native home service timed out')
        response = future.result()
        if response is None:
            raise RuntimeError('native home service returned no response')
        if response.accepted and paused:
            _home_pause(node, False, required=True)
        try:
            details = json.loads(response.reason)
        except (TypeError, ValueError):
            details = {'reason': response.reason}
        report = {
            'accepted': bool(response.accepted),
            'decision': response.decision,
            'request_id': response.request_id,
            'task_scope_id': scope,
            'details': details,
            'can_tx_owner': 'existing_native_manual_gamepad',
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if response.accepted else 3
    except Exception as exc:
        print(json.dumps({'accepted': False, 'decision': 'HOME_REQUEST_FAILED',
                          'error_type': type(exc).__name__, 'reason': str(exc)},
                         ensure_ascii=False, indent=2), file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
