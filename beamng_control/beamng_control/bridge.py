#!/usr/bin/env python3

import sys
import json
import copy
from pathlib import Path
from distutils.version import LooseVersion

import beamngpy as bngpy
from beamngpy import BeamNGpy, Scenario, Vehicle

import numpy as np
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import Clock

from action_msgs.msg import GoalStatus

# For TF 
from geometry_msgs.msg import QuaternionStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import math

# ROS 2 packages
import beamng_msgs.srv as bng_srv
# from beamng_msgs.msg import StepAction, StepFeedback, StepResult #Later
from beamng_msgs.srv import SpawnVehicle, StartScenario, TeleportVehicle, ChangeSimulationState
# from beamng_msgs.srv import GetCurrentVehiclesInfo, GetScenarioState, SpawnVehicle, StartScenario, TeleportVehicle, ChangeSimulationState #Later
import beamng_msgs.msg as bng_msgs


from beamng_control.publishers import VehiclePublisher, NetworkPublisher, get_sensor_publisher
from beamng_control.sensorHelper import get_sensors_classical, get_sensors_automation

MIN_BNG_VERSION_REQUIRED = '0.31.0'
NODE_NAME = 'beamng_control'
HOST_IP = '192.168.1.105'  #ForTesting
PORT_NO = 64256  #ForTesting

def load_json(file_path): #ForTesting
# def load_json(file_name):
    # pkg_path = rospkg.RosPack().get_path(NODE_NAME)
    # file_path = Path(file_name).resolve()
    # relative_fp = Path(str(pkg_path)+'/'+str(file_path))
    relative_fp = Path(str(file_path))
    # if not file_path.is_file() and relative_fp.is_file():
    #     file_path = relative_fp
    file_path = relative_fp
    with file_path.open('r') as fh:
        content = json.load(fh)
    return content

class BeamNGBridge(Node):
    def __init__(self, host= HOST_IP, port=PORT_NO, sensor_paths=None):  #ForTesting
    # def __init__(self, host, port, sensor_paths=None):
        super().__init__('beamng_control') # type: ignore
        self.game_client = bngpy.BeamNGpy(host, port)
        try:
            self.game_client.open(listen_ip='*', launch=False, deploy=False)
            self.get_logger().info("Successfully connected to BeamNG.tech")
        except TimeoutError:
            self.get_logger().fatal("Could not establish connection, check whether BeamNG.tech is running.")
            sys.exit(1)

        self.running = False
        # self._setup_services()
        self._publishers = list()
        self._vehicle_publisher = None
        self._static_tf_frames = []
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        sensor_paths = ['/home/ubuntu22/ros2_ws/src/beamng_control/config/sensors.json'] #ForTesting
        self._setup_sensor_defs(sensor_paths)
        # self._stepAS = actionlib.ActionServer(self, f'{NODE_NAME}/step', bng_msgs.StepAction, execute_callback=self.step, auto_start=False) #Later
        # self.on_shutdown(lambda: self.on_shutdown()) #Later
        self._marker_idx = 0
        self.network_publisher = None

    def _setup_sensor_defs(self, sensor_paths):
        default_path = ['/home/ubuntu22/ros2_ws/src/beamng_control/config/sensors.json'] #ForTesting
        # default_path = ['/config/sensors.json']
        sensor_paths = default_path if not sensor_paths else sensor_paths
        self.get_logger().debug("sensor_paths %s" % sensor_paths)
        print("------------")
        print("------------")
        print ("sensor_paths",sensor_paths)
        print("------------")
        print("------------")
        sensor_defs = dict()
        for path in sensor_paths:
            s = load_json(path)
            sensor_defs.update(s)
        self._sensor_defs = sensor_defs
        self.get_logger().info(f'self._sensor_defs "{self._sensor_defs}".')

        

    def _setup_services(self):
        self.add_service('get_scenario_state', bng_srv.GetScenarioState, self.get_scenario_state)
        self.add_service('spawn_vehicle', bng_srv.SpawnVehicle, self.spawn_new_vehicle)
        self.add_service('start_scenario', bng_srv.StartScenario, self.start_scenario_from_req)
        self.add_service('get_current_vehicles', bng_srv.GetCurrentVehiclesInfo, self.get_current_vehicles)
        self.add_service('teleport_vehicle', bng_srv.TeleportVehicle, self.teleport_vehicle)
        self.add_service('pause', bng_srv.ChangeSimulationState, self.pause)
        self.add_service('resume', bng_srv.ChangeSimulationState, self.resume)

    def add_service(self, service_name, srv_type, func):
        service_name = f'{NODE_NAME}/{service_name}'
        srv = self.create_service(srv_type, service_name, func)
        self.get_logger().info(f'Added service "{service_name}".')

    def get_marker_idx(self):
        m = self._marker_idx
        self._marker_idx += 1
        return m


    def get_sensor_classical_from_dict(self, v_spec):
        vehicle = bngpy.Vehicle(v_spec['name'], v_spec['model'])
        sensor_collection = list()
        noise_sensors = list()

        if 'sensors_classical' in v_spec:
            for spec in v_spec['sensors_classical']:
                if 'base sensor' in spec:
                    noise_sensors.append(spec)
                    self.get_logger().debug(f'noise_sensors_classical: {noise_sensors}')
                else:
                    sensor_collection.append(spec)
                    self.get_logger().debug(f'sensors_classical: {sensor_collection}')

        
        for s_spec in sensor_collection:
            s_name = s_spec.pop('name')
            s_type = s_spec.pop('type')
            self.get_logger().debug(f'Attempting to set up {s_type} sensor.') 
            sensor = get_sensors_classical(s_type,
                                self._sensor_defs,
                                dyn_sensor_properties=s_spec)      
            vehicle.attach_sensor(s_name, sensor)
        self.get_logger().debug(f'sensors_classical: {sensor_collection}')
        
        return vehicle



    def get_stamped_static_tf_frame(self, translation, rotation, vehicle_name, sensor_name):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = vehicle_name
        static_transform_stamped.child_frame_id = f"{vehicle_name}_{sensor_name}"
        static_transform_stamped.header.stamp = rclpy.clock.Clock().now().to_msg() # type: ignore
        static_transform_stamped.transform.translation.x =  float(translation[0])
        static_transform_stamped.transform.translation.y =  float(translation[1])
        static_transform_stamped.transform.translation.z =  float(translation[2])
                
        quaternion_msg = self.quaternion_from_euler(0.0, float(rotation[0]), float(rotation[1]))
        alignment_quat = [0, 0, 0, 1]  # sets the forward direction as -y
        quaternion_msg = self.quaternion_multiply(alignment_quat, quaternion_msg)
        quaternion_msg /= np.linalg.norm(quaternion_msg)

        static_transform_stamped.transform.rotation.x = quaternion_msg[0]
        static_transform_stamped.transform.rotation.y = quaternion_msg[1]
        static_transform_stamped.transform.rotation.z = quaternion_msg[2]
        static_transform_stamped.transform.rotation.w = quaternion_msg[3]
        return static_transform_stamped
    


    def quaternion_from_euler(self,ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def quaternion_multiply(self, q0, q1):
        """
        Multiplies two quaternions.

        Input
        :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
        :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

        Output
        :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

        """
        # Extract the values from q0
        w0 = q0[0]
        x0 = q0[1]
        y0 = q0[2]
        z0 = q0[3]

        # Extract the values from q1
        w1 = q1[0]
        x1 = q1[1]
        y1 = q1[2]
        z1 = q1[3]

        # Computer the product of the two quaternions, term by term
        q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        # Create a 4 element array containing the final quaternion
        final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])
        # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
        return final_quaternion


    def set_sensors_automation_from_dict(self, scenario_spec, vehicle_list):
        for v_spec, vehicle in zip(scenario_spec['vehicles'], vehicle_list):
            sensor_collection = list()
            noise_sensors = list()
            if 'sensors_automation' in v_spec:
                for spec in v_spec['sensors_automation']:
                    if 'base sensor' in spec:
                        noise_sensors.append(spec)
                    else:
                        sensor_collection.append(spec)
            self.get_logger().info(f'sensors_automation {sensor_collection}')

            for s_spec in sensor_collection:
                dyn_spec = copy.deepcopy(s_spec)
                dyn_spec.pop("name")
                dyn_spec.pop("type")
                s_type = s_spec["type"]
                name = s_spec["name"]
                print(s_type)
                print(name)
                sensor, sensor_publisher = get_sensors_automation(
                    s_type,
                    self._sensor_defs,
                    bng=self.game_client,
                    vehicle=vehicle,
                    name=name,
                    dyn_sensor_properties=dyn_spec
                )
                if sensor_publisher is not None:
                    print("get_stamped_static_tf_frame")
                    static_sensor_frame = self.get_stamped_static_tf_frame(
                        
                        translation=s_spec['position'],
                        rotation=s_spec['rotation'],
                        vehicle_name=vehicle.vid,
                        sensor_name=name
                    )
                    print("static_sensor_frame")
                    print(static_sensor_frame)
                    self._static_tf_frames.append(static_sensor_frame)
                    print("_static_tf_frames")
                    print(self._static_tf_frames)
                    
                    # self._publishers.append(sensor_publisher(sensor, f"{NODE_NAME}/{vehicle.vid}/{name}", vehicle))#WIP




    def get_vehicle_from_dict(self, v_spec):
        # Your implementation
        pass




    def _scenario_from_json(self, file_name):
        try:
            scenario_spec = load_json(file_name)
            self.get_logger().info(f'Opening scenario {file_name}' )
        except FileNotFoundError:
            self.get_logger().error(f'file "{file_name}" does not exist, abort')
            return
        self.get_logger().info(json.dumps(scenario_spec))
        return scenario_spec


    def decode_scenario(self, scenario_spec):
        vehicle_list = list()
        scenario = bngpy.Scenario(scenario_spec.pop('level'), scenario_spec.pop('name'))
        for v_spec in scenario_spec['vehicles']:
            vehicle = self.get_sensor_classical_from_dict(v_spec)
            # self._vehicle_publisher = VehiclePublisher(vehicle, NODE_NAME) #WIP
            vehicle_list.append(vehicle)
            self.get_logger().info(f'vehicle  "{vehicle}".')
            scenario.add_vehicle(vehicle, pos=v_spec['position'], rot_quat=v_spec['rotation'])

        
        on_scenario_start = list()
        wp_key = 'weather_presets'
        if wp_key in scenario_spec:
            def weather_presets():
                self.game_client.set_weather_preset(scenario_spec[wp_key])
            on_scenario_start.append(weather_presets)
        if 'time_of_day' in scenario_spec:
            def tod():
                self.game_client.set_tod(scenario_spec['time_of_day'])
            on_scenario_start.append(tod)
        net_viz_key = 'network_vizualization'
        if net_viz_key in scenario_spec and scenario_spec[net_viz_key] == 'on':
            self.get_logger().info(f'net_viz_key "{net_viz_key}".')
            print(NODE_NAME)
            self._publishers.append(NetworkPublisher(self.game_client, NODE_NAME)) # type: ignore
        return scenario, on_scenario_start, vehicle_list



    def start_scenario(self, file_name):
        self._publishers = list()
        scenario_spec = self._scenario_from_json(file_name)
        self.get_logger().info(f'Started scenario "{scenario_spec}".')
        
        if not scenario_spec:
            return
        scenario, on_scenario_start, vehicle_list = self.decode_scenario(scenario_spec)
        scenario.make(self.game_client)
        self.game_client.load_scenario(scenario)
        self.game_client.start_scenario()
        self.set_sensors_automation_from_dict(scenario_spec, vehicle_list)
        for hook in on_scenario_start:
            hook()
        if 'mode' in scenario_spec and scenario_spec['mode'] == 'paused':
            self.game_client.pause()
        else:
            self.game_client.resume()
        self.get_logger().info(f'Started scenario "{scenario.name}".')
        self.running = True



    def get_scenario_state(self, req, response):
        # Your implementation
        pass

    def spawn_new_vehicle(self, req, response):
        # Your implementation
        pass

    def start_scenario_from_req(self, req, response):
        # Your implementation
        pass

    def get_current_vehicles(self, req, response):
        # Your implementation
        pass

    def teleport_vehicle(self, req, response):
        # Your implementation
        pass

    def pause(self, req, response):
        # Your implementation
        pass

    def resume(self, req, response):
        # Your implementation
        pass

    def step(self, goal_handle):
        # Your implementation
        pass


    def get_roads(self):
        roads = self.game_client.get_roads()
        road_spec = {}
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                road_spec[r_id] = self.game_client.get_road_edges(r_id)

        network = list()
        for r_id in road_spec.keys():
            left = list()
            right = list()
            for r_point in road_spec[r_id]:
                left.append(r_point['left'])
                right.append(r_point['right'])
            if left:
                network.append(left)
            if right:
                network.append(right)
        # print(network) 
        return network

    # def work(self):
    #     current_time = rclpy.clock.Clock().now().to_msg()  # type: ignore
    #     # current_time = self.get_clock().now().to_msg()
    #     # current_time = Clock().now() #ForTesting

    #     for static_tf in self._static_tf_frames:
    #         static_tf.header.stamp = current_time
    #         self._static_tf_broadcaster.sendTransform(static_tf)

    #     if self._vehicle_publisher is not None:
    #         self._vehicle_publisher.publish(current_time)

    #     for pub in self._publishers:
    #         pub.publish(current_time)

# ...

    def work(self):
        ros_rate = 10
        rate = self.create_rate(ros_rate)

        while rclpy.ok() and self.running:
            current_time = rclpy.clock.Clock().now().to_msg()

            for static_tf in self._static_tf_frames:
                static_tf.header.stamp = current_time
                self._static_tf_broadcaster.sendTransform(static_tf)

            if self._vehicle_publisher is not None:
                self._vehicle_publisher.publish(current_time)

            for pub in self._publishers:
                pub.publish(current_time)

            rate.sleep()

# ...


    def on_shutdown(self):
        self.get_logger().info("Shutting down beamng_control/bridge.py node")
        self.game_client.disconnect()
        # node.destroy_node()
        # rclpy.shutdown()
        
def main():
    rclpy.init()
    node = BeamNGBridge()#ForTesting
    # node = rclpy.create_node(NODE_NAME) #WIP (not working with FT)
    # node = rclpy.create_node(NODE_NAME, anonymous=True, log_level=rclpy.logging.LoggingSeverity.DEBUG)  
    # node_name = node.get_name() (not working with FT)
    # node.get_logger().info(f'Started node "{node_name}".')
    # print(f'Started node "{node_name}".') #ForTestiong (not working with FT)

    available_version = bngpy.__version__
    if LooseVersion(MIN_BNG_VERSION_REQUIRED) > LooseVersion(available_version):
        node.get_logger().fatal(f'This package requires at least BeamNGpy version {MIN_BNG_VERSION_REQUIRED}, but the available version is {available_version}, aborting process.')
        sys.exit(1)

    # host = node.get_parameter("~host").get_parameter_value().string_value
    # port = node.get_parameter("~port").get_parameter_value().integer_value
    port = PORT_NO #ForTesting
    host= HOST_IP #ForTesting
    if host is None or port is None:
        node.get_logger().fatal("No host or port specified on the parameter server to connect to Beamng.tech")
        sys.exit()
    bridge = BeamNGBridge(host, port)
    if len(sys.argv) == 2:
        node.get_logger().debug(f'Detected optional input, creating scenario from json: "{sys.argv[1]}"')
        bridge.start_scenario(sys.argv[1])
    scenario_var="/home/ubuntu22/ros2_ws/src/beamng_control/config/scenarios/west_coast_with_all_sensors.json" #ForTesting
    bridge.start_scenario(scenario_var) #ForTesting
    bridge.work()
    # bridge.get_roads()

    
    rclpy.spin(bridge)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
