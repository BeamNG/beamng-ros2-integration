import socket
from typing import Any, Dict, Optional

import beamng_msgs.msg as beamng_msgs
import numpy as np
from beamng_ros2.publishers.base import VehiclePublisher
from beamngpy.vehicle import Vehicle
from rclpy.node import Node


class CouplingPublisher(VehiclePublisher):
    DELAY_OFFSET_SEC = 10.0

    def __init__(self, config: Dict[str, Any], node: Node, vehicle: Vehicle) -> None:
        super().__init__()
        self.name = "cosim"
        self.update_time = config["time3rdParty"] / 2
        self.vehicle = vehicle

        self.recv_ip = config["udpReceiveIP"]
        self.recv_port = config["udpReceivePort"]
        send_ip = config["udpSendIP"]
        send_port = config["udpSendPort"]
        self.signals_to = config["signalsTo"]
        self.signals_from = config["signalsFrom"]
        self.sig_map_from = {
            f'{s["groupName"]}.{s["name"]}': i for i, s in enumerate(self.signals_from)
        }
        self.sig_map_to = {
            f'{s["groupName"]}.{s["name"]}': i for i, s in enumerate(self.signals_to)
        }
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(True)
        self.socket.bind((send_ip, send_port))
        self._send_buffer = np.zeros((1 + len(self.signals_from)), dtype=np.float64)
        self._recv_buffer = bytearray((1 + len(self.signals_to)) * 8)

        self._publisher = node.create_publisher(
            beamng_msgs.CosimulationData, "~/cosim", 0
        )
        self._subscription = node.create_subscription(
            beamng_msgs.CosimulationData, "~/cosim/input", self.subscription_callback, 0
        )
        self._subscription_name = node.create_subscription(
            beamng_msgs.CosimulationInput,
            "~/cosim/input/by_name",
            self.subscription_callback_namevalue,
            0,
        )
        self._clock = node.get_clock()
        self._timer = node.create_timer(self.update_time, self.publish)
        self.connected = False
        self.shutdown = False

    def clear_send_buffer(self):
        self._send_buffer[1:].fill(0)

    def update_send_buffer(self, name: str, value: float):
        self._send_buffer[self.sig_map_from[name] + 1] = value

    def msg_type(self):
        return beamng_msgs.CosimulationData

    def send(self) -> None:
        """
        Send a msg to BeamNG on the send address.
        """
        self.socket.sendto(self._send_buffer.tobytes(), (self.recv_ip, self.recv_port))

    def recv(self) -> Optional[np.ndarray]:
        count, addr = self.socket.recvfrom_into(self._recv_buffer)
        if not self.connected:
            self.socket.connect(addr)
            self.connected = True
        if count == 0:
            return None

        unpacked_msg = np.frombuffer(self._recv_buffer, dtype=np.float64)
        self._send_buffer[0] = unpacked_msg[0]  # id of the message
        return unpacked_msg[1:]

    def subscription_callback(self, data: beamng_msgs.CosimulationData):
        assert len(data.data) == len(self._send_buffer) - 1
        self._send_buffer[1:] = data.data

    def subscription_callback_namevalue(self, data: beamng_msgs.CosimulationInput):
        for i in range(len(data.name)):
            self.update_send_buffer(data.name[i], data.value[i])

    def publish(self):
        if self.shutdown:
            return
        data = self.recv()
        if data is not None:
            self._publisher.publish(beamng_msgs.CosimulationData(data=data))
            self.send()

    def stop(self):
        self.shutdown = True
        self.socket.shutdown(socket.SHUT_RDWR)
