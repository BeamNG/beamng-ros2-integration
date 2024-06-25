"""
A minimal working example client which uses the StartScenario service of the BeamNGBridge node
to spawn the default scenario (``/config/scenarios/example_tech_ground.json``).
"""

import rclpy
from beamng_msgs.srv import StartScenario
from rclpy.node import Node


class ExampleClientAsync(Node):
    def __init__(self):
        super().__init__("beamng_example_client_async")
        self.cli = self.create_client(StartScenario, "/beamng_bridge/start_scenario")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = StartScenario.Request()

    def send_request(self):
        """
        Sends the ``StartScenario`` request to the BeamNGBridge node.
        """
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    """
    The entrypoint of the example client.

    Args:
        args: list of command-line arguments for ``rclpy``
    """

    rclpy.init(args=args)

    minimal_client = ExampleClientAsync()
    response = minimal_client.send_request()

    input("Press Enter to exit...")
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
