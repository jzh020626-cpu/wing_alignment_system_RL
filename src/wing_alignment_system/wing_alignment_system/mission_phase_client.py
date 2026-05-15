#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def _service_name_for_phase(phase_name: str) -> str:
    phase = str(phase_name or '').lower().strip()
    mapping = {
        'approach': '/mission/start_approach',
        'slide_align': '/mission/start_slide_align',
        'level_recenter': '/mission/start_level_recenter',
        'transport': '/mission/start_transport',
        'reset_to_standby': '/mission/reset_to_standby',
        'status': '/mission/get_status',
    }
    if phase not in mapping:
        raise ValueError(f'Unsupported phase command: {phase_name}')
    return mapping[phase]


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Trigger managed mission phases.')
    parser.add_argument(
        '--phase',
        required=True,
        choices=['approach', 'slide_align', 'level_recenter', 'transport', 'reset_to_standby', 'status'],
        help='Managed mission phase command to send.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=5.0,
        help='Service wait timeout in seconds.',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    service_name = _service_name_for_phase(args.phase)

    rclpy.init(args=None)
    node = Node('mission_phase_client')
    cli = node.create_client(Trigger, service_name)

    try:
        if not cli.wait_for_service(timeout_sec=max(0.1, float(args.timeout_sec))):
            print(f'service not available: {service_name}', file=sys.stderr)
            return 1

        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.1, float(args.timeout_sec)))
        if not future.done():
            print(f'service call timed out: {service_name}', file=sys.stderr)
            return 1

        response = future.result()
        if response is None:
            print(f'service call failed: {service_name}', file=sys.stderr)
            return 1

        print(response.message)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
