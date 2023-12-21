#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

import json

from pathlib import Path
from distutils.version import LooseVersion

from beamng_msgs.msg import VehicleControl  
import beamngpy as bngpy
from beamngpy import BeamNGpy, Scenario, Vehicle


MIN_BNG_VERSION_REQUIRED = '0.31.0'
NODE_NAME = 'beamng_agent'
HOST_IP = '192.168.1.105'  #ForTesting
PORT_NO = 64256  #ForTesting
VEHICLE = "ego_vehicle" #ForTesting


# class BeamNGAgentNode():
    # def __init__(self, vehicle_id):
        # self.node = rclpy.create_node(NODE_NAME)
        # params = self.node.get_parameters(parameter_names=['beamng'])
        # self.game_client = bngpy.BeamNGpy(params['host'].value, params['port'].value)

class BeamNGAgentNode(Node):
    def __init__(self, host=HOST_IP, port=PORT_NO, vehicle_id=VEHICLE ):  #ForTesting
        super().__init__('beamng_agent') # type: ignore
        self.game_client = bngpy.BeamNGpy(host, port) #ForTesting

        try:
            # self.game_client.open(listen_ip='*', launch=False)
            self.game_client.open(listen_ip='*', launch=False, deploy=False)
            print("Successfully connected to BeamNG.tech.")
            # self.node.get_logger().info("Successfully connected to BeamNG.tech.")
        except TimeoutError:
            print("Could not establish game connection, check whether BeamNG.tech is running.")
            # self.node.get_logger().error("Could not establish game connection, check whether BeamNG.tech is running.")
            sys.exit(1)
            
        self.running = False
        current_vehicles = self.game_client.get_current_vehicles()

        ass_msg = f"no vehicle with id {vehicle_id} exists"
        assert vehicle_id in current_vehicles.keys(), ass_msg
        self.vehicle_client = current_vehicles[vehicle_id]
        try:
            self.vehicle_client.connect(self.game_client)
            vid = self.vehicle_client.vid
            print(f'Successfully connected to vehicle client with id {vid}')
            # self.node.get_logger().info(f'Successfully connected to vehicle client with id {vid}')
            self.running = True
        except TimeoutError:
            # self.node.get_logger().fatal("Could not establish vehicle connection, system exit.")
            print("Could not establish vehicle connection, system exit.")
            sys.exit(1)

        control_topic = 'control'
        print ("setup completed")
        self.subscription = self.create_subscription(
            VehicleControl,
            control_topic,
            self.send_control_signal,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning
        
        # self.subscription = self.node.create_subscription(
        #     VehicleControl,
        #     control_topic,
        #     self.send_control_signal
        #     # 10  # Queue size
        # )
    #     # self.node.get_logger().info(f'Subscribing to "{control_topic}" for vehicle control')
    #     # self.node.get_logger().info(f'Shutting down node "{NODE_NAME}"')
        print(f'Subscribing to "{control_topic}" for vehicle control')
        print(f'Shutting down node "{NODE_NAME}"')

    def send_control_signal(self, signal):
        self.vehicle_client.control(
            steering=signal.steering,
            throttle=signal.throttle,
            brake=signal.brake,
            parkingbrake=signal.parkingbrake,
            clutch=signal.clutch,
            gear=signal.gear
        )

    def on_shutdown(self):
        self.vehicle_client.disconnect()
        self.game_client.disconnect()


    def work(self):
        ros_rate = 10
        rate = self.create_rate(ros_rate)
        while rclpy.ok() and self.running:
            print("running", self.running)
            rate.sleep()




# def main(args=None):
#     rclpy.init(args=args)
def main():
    rclpy.init()
    node = BeamNGAgentNode() 
    # node = rclpy.create_node(NODE_NAME) #WIP

    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        node.get_logger().fatal(f'This package requires at least BeamNGpy version {MIN_BNG_VERSION_REQUIRED}, but the available version is {available_version}, aborting process.')

        sys.exit(1)
    vehicle_id = VEHICLE
    port = PORT_NO #ForTesting
    host= HOST_IP #ForTesting
    
    argv = sys.argv
    if host is None or port is None:
        node.get_logger().fatal("No host or port specified on the parameter server to connect to Beamng.tech")
        sys.exit()
    agent = BeamNGAgentNode(vehicle_id, port, host)
    agent.work()
    

    # if len(argv) == 2:
    #     vehicle_id = argv[1]
    # else:
    #     print("No Vehicle ID given, shutting down node.")
    #     sys.exit(1)

    # try:
    #     agent = BeamNGAgentNode(vehicle_id, port, host)
    #     print("tring to connect to BeamNG.Tech ")
    #     rclpy.spin(agent)
    #     rclpy.shutdown()

    #     # rclpy.spin(node.node)
    # except KeyboardInterrupt:
    #     print("KeyboardInterrupt",KeyboardInterrupt)
    #     # pass
    # # finally:
    # #     # node.on_shutdown()
    # #     rclpy.shutdown()

if __name__ == '__main__':
    main()
