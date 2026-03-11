"""
One-shot node that waits for the beamng_bridge start_scenario service and calls it
with the configured scenario path. Exits after the call (used by launch files).
"""

import sys
import rclpy
from beamng_msgs.srv import StartScenario
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)

    node = Node("start_scenario")
    node.declare_parameter("scenario", "/config/scenarios/drone_scenario.json")
    scenario = node.get_parameter("scenario").value

    cli = node.create_client(StartScenario, "/beamng_bridge/start_scenario")
    node.get_logger().info(
        f"Waiting for /beamng_bridge/start_scenario (scenario={scenario})..."
    )
    while not cli.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            return

    req = StartScenario.Request()
    req.path_to_scenario_definition = scenario

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    result = future.result()
    node.destroy_node()
    rclpy.shutdown()

    if result is not None:
        print(f"[start_scenario] Scenario started: {scenario}")
    else:
        print(f"[start_scenario] Failed to start scenario: {scenario}", file=sys.stderr)


if __name__ == "__main__":
    main()
