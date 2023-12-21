#!/usr/bin/env python3


# import tf
# tfb = tf2_ros.TransformBroadcaster()
# from tf.transformations import quaternion_from_euler, quaternion_multiply
# import tf.transformations as tf_transformations


import sys 
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from abc import ABC, abstractmethod
import numpy as np
import math
from distutils.version import LooseVersion

import builtin_interfaces.msg
import geometry_msgs.msg as geom_msgs
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Duration
import std_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Point, Point32, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Range, Image, Imu, PointField
# import sensor_msgs.point_cloud2 as pc2
# from pcl_msgs.msg import PointCloud2
# from sensor_msgs.point_cloud2 import PointCloud2
import tf2_ros 
from tf2_ros import Buffer, TransformListener
# from tf2_ros import Buffer, TransformListener, do_transform_cloud
# import tf2_ros as tf
import numpy as np
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from tf2_geometry_msgs import do_transform_cloud
import tf2_geometry_msgs
from rclpy.qos import qos_profile_sensor_data
import ros2_numpy 
# np.float = np.float64  # temp fix for following import
numpy_float64 = np.float64(3.14)
from cv_bridge import CvBridge, CvBridgeError
import threading

import beamngpy.sensors as bng_sensors
import beamng_msgs.msg as bng_msgs
# from beamngpy.noise import RandomImageNoise, RandomLIDARNoise

# import numpy as np
# from geometry_msgs.msg import Quaternion
# from tf.transformations import quaternion_from_euler, quaternion_multiply

# # Create a quaternion from Euler angles (roll, pitch, yaw)
# roll, pitch, yaw = 0.1, 0.2, 0.3
# euler_angles = np.array([roll, pitch, yaw])
# quaternion = quaternion_from_euler(*euler_angles)

# # Multiply two quaternions
# q1 = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
# q_result = quaternion_multiply(q1, quaternion)

# # Convert the result back to a ROS Quaternion message
# ros_quaternion = Quaternion(*q_result)

# print("ROS Quaternion:", ros_quaternion)



def get_sensor_publisher(sensor):
    sensor_mapping = {
        bng_sensors.State: StatePublisher,
        bng_sensors.Timer: TimerPublisher,
        bng_sensors.Damage: DamagePublisher,
        bng_sensors.GForces: GForcePublisher,
        bng_sensors.IMU: IMUPublisher,
        bng_sensors.Ultrasonic: UltrasonicPublisher,
        bng_sensors.Electrics: ElectricsPublisher,
        bng_sensors.Camera: CameraPublisher,
        bng_sensors.Lidar: LidarPublisher,
        # RandomImageNoise: CameraPublisher,
        # RandomLIDARNoise: LidarPublisher
    }
    for k, v in sensor_mapping.items():
        if isinstance(sensor, k):
            return v
    print(f'Could not identify publisher for {type(sensor)}') #ForTesting


class BNGPublisher(ABC):

    @abstractmethod
    def publish(self, current_time):
        pass



class SensorDataPublisher(BNGPublisher):
# class SensorDataPublisher(Node, BNGPublisher):
    def __init__(self, sensor, topic_id, msg_type):
        # super().__init__(self, Node)
        BNGPublisher.__init__(self)
        print(f'publishing to {topic_id}')  #ForTesting 
        # self.get_logger().debug(f'publishing to {topic_id}')
        self._sensor = sensor
        # self._pub = self.create_publisher(topic_id, msg_type)
        # self._pub = self.create_publisher(topic_id, msg_type , queue_size=1)
        # self._pub = self.create_publisher(Marker, topic_id,msg_type ) #ForTesting
        # rclpy.init()
        # self.node = rclpy.create_node(topic_id)
        # self._pub = self.node.create_publisher(msg_type, topic_id, 1) #WIP
        # self._pub = self.create_publisher(msg_type, topic_id, 1) 

        self.current_time = rclpy.clock.Clock().now().to_msg() # type: ignore
        # self.current_time = Clock().now() #ForTesting
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self):
        pass

    def publish(self, current_time):
        self.current_time = current_time
        msg = self._make_msg()
        self._pub.publish(msg)

class StatePublisher(SensorDataPublisher):
    def __init__(self, sensor, topic_id):
        super().__init__(sensor, topic_id, bng_msgs.StateSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.StateSensor()
        
        # msg.position = Vector3(x=data['pos']['x'], y=data['pos']['y'], z=data['pos']['z'])
        # msg.velocity = Vector3(x=data['vel']['x'], y=data['vel']['y'], z=data['vel']['z'])
        msg.position = data['pos']
        msg.velocity = data['vel']        
        msg.front = data['front']
        msg.up = data['up']
        msg.dir = data['dir']
        
        return msg

class TimerPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.TimeSensor)

    # def _make_msg(self): 
    #     msg = bng_msgs.TimeSensor()
    #     data = self._sensor.data
    #     seconds = int(data['time'])
    #     nseconds = (seconds - seconds//1) * 1e9
    #     msg.beamng_simulation_time.data.set(int(seconds), 0)
    #     return msg
    
    
    def _make_msg(self):
        msg = bng_msgs.TimeSensor()
        data = self._sensor.data
        seconds = int(data['time'])
        nseconds = int((data['time'] - seconds) * 1e9)

        # Set the time in the ROS2 Time message
        msg.beamng_simulation_time.sec = seconds
        msg.beamng_simulation_time.nanosec = nseconds
        return msg

class DamagePublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.DamageSensor)

    def _make_msg(self):
        msg = bng_msgs.DamageSensor()
        data = self._sensor.data
        for k, v in data['deform_group_damage'].items():
            msg.inv_max_events.append(k)
            # msg.deformgroup_id.append(v['invMaxEvents']) #WIP
            msg.damage.append(v['damage'])
            msg.event_count.append(v['eventCount'])
            msg.max_events.append(v['maxEvents'])
        if data['part_damage']:
            for k, v in data['part_damage'].items():
                # msg.part_id.append(k) #WIP
                # msg.name.append(v['name'])  # todo what is the diff to #WIP
                msg.part_damage.append(v['damage'])  # todo is it in %?
        return msg




class GForcePublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.GForceSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.GForceSensor()
        msg.gx = data['gx']
        msg.gy = data['gy']
        msg.gz = data['gz']
        msg.gx2 = data['gx2']
        msg.gy2 = data['gy2']
        msg.gz2 = data['gz2']
        return msg


class IMUPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         Imu)

    def _make_msg(self):
        data = self._sensor.data
        msg = Imu()
        msg.orientation = geom_msgs.Quaternion(0, 0, 0, 0)
        msg.orientation_covariance = [-1, ] * 9
        msg.angular_velocity = geom_msgs.Vector3(*[data[x] for x in ['aX', 'aY', 'aZ']])
        msg.angular_velocity_covariance = [-1, ] * 9
        msg.linear_acceleration = geom_msgs.Vector3(*[data[x] for x in ['gX', 'gY', 'gZ']])
        msg.linear_acceleration_covariance = [-1, ] * 9
        return msg


class UltrasonicPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, Range)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self) #ForTesting
        
        # self.listener = tf2_ros.TransformListener()
        # self.listener = tf.TransformListener() 
        sensor_name = topic_id.split("/")[-1]
        self.frame_USSensor_sensor = f'{vehicle.vid}_{sensor_name}'
        # self.current_time = Clock().now()#ForTesting
        self.current_time =  rclpy.clock.Clock().now().to_msg() # type: ignore

    def _make_msg(self):      
        data = self._sensor.poll()
        USSensor_msg = Range()
        USSensor_msg.radiation_type = Range.ULTRASOUND
        USSensor_msg.header.frame_id =  self.frame_USSensor_sensor 
        USSensor_msg.header.stamp = self.current_time
        USSensor_msg.field_of_view = 0.1
        USSensor_msg.min_range = 0.15
        USSensor_msg.max_range = 2.5
        USSensor_msg.range =  data['distance']

        try:
            transform = self.tf_buffer.lookup_transform(self.frame_map, self.frame_USSensor_sensor, USSensor_msg.header.stamp) #ForTesting
        except:

            print(f'No transform between {self.frame_map} and {self.frame_USSensor_sensor} available')
        
        # (trans_map, _) = self.listener.lookupTransform(self.frame_map, self.frame_USSensor_sensor, USSensor_msg.header.stamp) #ForTesting 
        
        # try:
        #     (trans_map, _) = self.listener.lookupTransform(self.frame_map, self.frame_USSensor_sensor, USSensor_msg.header.stamp)
        # # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #     self.get_logger().warning(f'No transform between {self.frame_map} and '
        #                   f'{self.frame_USSensor_sensor} available with exception: {e}')
            
        return USSensor_msg


class ElectricsPublisher(SensorDataPublisher):

    def __init__(self, sensor, topic_id):
        super().__init__(sensor,
                         topic_id,
                         bng_msgs.ElectricsSensor)

    def _make_msg(self):
        data = self._sensor.data
        msg = bng_msgs.ElectricsSensor()
        msg.acc_x_smooth = data['acc_x_smooth']
        msg.acc_y_smooth = data['acc_y_smooth']
        msg.acc_z_smooth = data['acc_z_smooth']
        msg.parking_brake_input = data['parking_brake_input']
        msg.parking_brake_light = data['parking_brake_light']
        msg.brake_light_signal_r = data['brake_light_signal_r']
        msg.brake_light_signal_l = data['brake_light_signal_l']
        msg.is_yc_brake_active = data['is_yc_brake_active']
        msg.brake = data['brake']
        msg.is_tc_brake_active = data['is_tc_brake_active']
        msg.brake_lights = data['brake_lights']
        msg.parking_brake = data['parking_brake']
        msg.brake_input = data['brake_input']
        msg.is_abs_brake_active = data['is_abs_brake_active']
        msg.parking = data['parking']
        msg.hazard = data['hazard']
        msg.hood_catch_coupler_not_attached = data['hood_catch_coupler_not_attached']
        msg.door_fl_coupler_not_attached = data['door_fl_coupler_not_attached']
        msg.oil = data['oil']
        msg.low_beam = data['low_beam']
        msg.high_beam = data['high_beam']
        msg.low_high_beam = data['low_high_beam']
        msg.high_beam_wigwag_l = data['high_beam_wigwag_l']
        msg.high_beam_wigwag_r = data['high_beam_wigwag_r']
        msg.low_high_beam_signal_r = data['low_high_beam_signal_r']
        msg.low_high_beam_signal_l = data['low_high_beam_signal_l']
        msg.has_abs = data['has_abs']
        msg.throttle = data['throttle']
        msg.reverse = data['reverse']
        msg.reverse_wigwag_r = data['reverse_wigwag_r']
        msg.reverse_wigwag_l = data['reverse_wigwag_l']
        msg.ignition_level = data['ignition_level']
        msg.gear = data['gear']
        msg.gearbox_mode = data['gearbox_mode']
        msg.gear_index = data['gear_index']
        msg.gear_a = data['gear_a']
        msg.gear_m = data['gear_m']
        msg.gear_mode_index = data['gear_mode_index']
        msg.min_gear_index = data['min_gear_index']
        msg.max_gear_index = data['max_gear_index']
        msg.nop = data['nop']
        msg.lockup_clutch_ratio = data['lockup_clutch_ratio']
        msg.turbo_rpm_ratio = data['turbo_rpm_ratio']
        msg.turbo_spin = data['turbo_spin']
        msg.turbo_rpm = data['turbo_rpm']
        msg.idle_rpm = data['idle_rpm']
        msg.turn_signal = data['turn_signal']
        msg.light_bar = data['light_bar']
        msg.max_rpm = data['max_rpm']
        msg.clutch = data['clutch']
        msg.has_tcs = data['has_tcs']
        msg.door_rr_coupler_not_attached = data['door_rr_coupler_not_attached']
        msg.door_rl_coupler_not_attached = data['door_rl_coupler_not_attached']
        msg.door_fr_coupler_not_attached = data['door_fr_coupler_not_attached']
        msg.hood_latch_coupler_not_attached = data['hood_latch_coupler_not_attached']
        msg.tcs = data['tcs']
        msg.esc = data['esc']
        msg.dse_warning_pulse = data['dse_warning_pulse']
        msg.steering_unassisted = data['steering_unassisted']
        msg.steering = data['steering']
        msg.steering_input = data['steering_input']
        msg.clutch_input = data['clutch_input']
        msg.throttle_input = data['throttle_input']
        msg.horn = data['horn']
        msg.low_pressure = data['low_pressure']
        msg.has_esc = data['has_esc']
        msg.abs_active = data['abs_active']
        msg.clutch_ratio = data['clutch_ratio']
        msg.fog_lights = data['fog_lights']
        msg.fuel_capacity = data['fuel_capacity']
        msg.radiator_fan_spin = data['radiator_fan_spin']
        msg.trip = data['trip']
        msg.airflow_speed = data['airflow_speed']
        msg.turbo_boost = data['turbo_boost']
        msg.turbo_boost_max = data['turbo_boost_max']
        msg.virtual_airspeed = data['virtual_airspeed']
        msg.driveshaft = data['driveshaft']
        msg.smooth_shift_logic_av = data['smooth_shift_logic_av']
        msg.rpm_spin = data['rpm_spin']
        msg.boost = data['boost']
        msg.boost_max = data['boost_max']
        msg.altitude = data['altitude']
        msg.odometer = data['odometer']
        msg.rpm = data['rpm']
        msg.fuel = data['fuel']
        msg.airspeed = data['airspeed']
        msg.wheelspeed = data['wheelspeed']
        msg.avg_wheel_av = data['avg_wheel_av']
        msg.engine_load = data['engine_load']
        msg.exhaust_flow = data['exhaust_flow']
        msg.fuel_volume = data['fuel_volume']
        msg.oil_temperature = data['oil_temperature']
        msg.rpm_tacho = data['rpm_tacho']
        msg.water_temperature = data['water_temperature']
        msg.two_step = data['two_step']
        msg.running = data['running']
        msg.ignition = data['ignition']
        msg.freeze_state = data['freeze_state']
        msg.left_signal = data['left_signal']
        msg.hazard_signal = data['hazard_signal']
        msg.esc_active = data['esc_active']
        msg.check_engine = data['check_engine']
        msg.low_fuel = data['low_fuel']
        msg.right_signal = data['right_signal']
        msg.tcs_active = data['tcs_active']
        msg.is_shifting = data['is_shifting']
        msg.signal_l = data['signal_l']
        msg.signal_r = data['signal_r']
        msg.headlights = data['headlights']
        msg.engine_throttle = data['engine_throttle']
        msg.tailgate_coupler_not_attached = data['tailgate_coupler_not_attached']
        msg.lights = data['lights']
        msg.engine_running = data['engine_running']
        msg.abs = data['abs']
        return msg


class CameraDataPublisher:

    def __init__(self, sensor, topic_id, msg_type):
        print(f'publishing to {topic_id}') #ForTesting
        # self.get_logger().debug(f'publishing to {topic_id}')
        self._sensor = sensor
        self._pub = self.create_publisher(topic_id,
                                    msg_type,
                                    queue_size=1)
        
        # self.current_time = Clock().now()#ForTesting
        self.current_time = rclpy.clock.Clock().now() # type: ignore
        self.frame_map = 'map'

    @abstractmethod
    def _make_msg(self, data):
        pass

    def publish(self, current_time, data):
        self.current_time = current_time
        msg = self._make_msg(data)
        self._pub.publish(msg)


class ColorImgPublisher(CameraDataPublisher):

    def __init__(self, sensor, topic_id, cv_helper, data_descriptor):
        super().__init__(sensor,
                         topic_id,
                         Image)

        self._cv_helper = cv_helper
        self._data_descriptor = data_descriptor

    def _make_msg(self, data):
        img = data[self._data_descriptor]
        if img is not None:
            img = np.array(img.convert('RGB'))
            img = img[:, :, ::-1].copy()
        else:
            img = np.zeros_like(data['colour'].convert('RGB'))
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'bgr8')
        except CvBridgeError as e:
            print(e)#ForTesting
            # self.get_logger().error(e)
        return img


class DepthImgPublisher(CameraDataPublisher):

    def __init__(self, sensor, topic_id, cv_helper):
        super().__init__(sensor,
                         topic_id,
                         Image)
        self._cv_helper = cv_helper

    def _make_msg(self, data):
        img = data['depth']
        near, far = self._sensor.near_far_planes
        img = (np.array(img) - near) / far * 255
        img = img.astype(np.uint8)
        try:
            img = self._cv_helper.cv2_to_imgmsg(img, 'mono8')
        except CvBridgeError as e:
            print(e)#ForTesting
            # self.get_logger().error(e)
        return img

# Not supported anymore 
# class BBoxImgPublisher(CameraDataPublisher):

#     def __init__(self, sensor, topic_id, cv_helper, vehicle):
#         super().__init__(sensor,
#                          topic_id,
#                          Image)
#         self._cv_helper = cv_helper
#         self._vehicle = vehicle
#         self._classes = None

#     def _update_data_with_bbox(self, data):
#         if self._classes is None:
#             annotations = self._vehicle.bng.get_annotations()
#             self._classes = self._vehicle.bng.get_annotation_classes(annotations)
#         bboxes = bng_sensors.Camera.extract_bboxes(data['annotation'],
#                                                    data['instance'],
#                                                    self._classes)
#         bboxes = [b for b in bboxes if b['class'] == 'CAR']
#         print(f'bboxes: {bboxes}')#ForTesting
#         # self.get_logger().debug(f'bboxes: {bboxes}')
#         bbox_img = bng_sensors.Camera.draw_bboxes(bboxes,
#                                                   data['colour'],
#                                                   width=3)
#         return bbox_img

#     def _make_msg(self, data):
#         img = self._update_data_with_bbox(data)
#         img = img.convert('RGB')
#         img = np.array(img)
#         img = img[:, :, ::-1].copy()
#         try:
#             img = self._cv_helper.cv2_to_imgmsg(img, 'bgr8')
#         except CvBridgeError as e:
#             print(e)#ForTesting
#             # self.get_logger().error(e)
#         return img


class CameraPublisher(BNGPublisher):

    def __init__(self, sensor, topic_id, vehicle):
        self._sensor = sensor
        self._cv_helper = CvBridge()
        self._publishers = list()
        if self._sensor.is_render_colours:
            color_topic = '/'.join([topic_id, 'colour'])
            pub = ColorImgPublisher(sensor,
                                    color_topic,
                                    self._cv_helper,
                                    'colour')
            self._publishers.append(pub)
        if self._sensor.is_render_depth:
            depth_topic = '/'.join([topic_id, 'depth'])
            pub = DepthImgPublisher(sensor,
                                    depth_topic,
                                    self._cv_helper)
            self._publishers.append(pub)
        if self._sensor.is_render_annotations:
            annotation_topic = '/'.join([topic_id, 'annotation'])
            pub = ColorImgPublisher(sensor,
                                    annotation_topic,
                                    self._cv_helper,
                                    'annotation')
            self._publishers.append(pub)
        if self._sensor.is_render_instance:
            inst_ann_topic = '/'.join([topic_id, 'instance'])
            pub = ColorImgPublisher(sensor,
                                    inst_ann_topic,
                                    self._cv_helper,
                                    'instance')
            self._publishers.append(pub)
        if self._sensor.bbox:
            print("bbox isn't supported anymore")
            # bbox_topic = '/'.join([topic_id, 'bounding_box'])
            # pub = BBoxImgPublisher(sensor,
            #                        bbox_topic,
            #                        self._cv_helper,
            #                        vehicle)
            # self._publishers.append(pub)

    def publish(self, current_time):
        if self._sensor.is_render_instance:
            data = self._sensor.get_full_poll_request()
        else:
            data = self._sensor.poll()
        for pub in self._publishers:
            pub.current_time = current_time
            pub.publish(current_time, data)


# class LidarPublisher(SensorDataPublisher):

#     def __init__(self, sensor, topic_id, vehicle):
#         super().__init__(sensor, topic_id, PointCloud2)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         sensor_name = topic_id.split("/")[-1]
#         self.frame_lidar_sensor = f'{vehicle.vid}_{sensor_name}'

#     def _make_msg(self):
#         header = std_msgs.msg.Header()
#         header.frame_id = self.frame_lidar_sensor
#         header.stamp = self.current_time

#         readings_data = self._sensor.poll()
#         points = np.array(readings_data['pointCloud'])
#         colours = readings_data['colours']

#         pointcloud_fields = [('x', np.float32),
#                              ('y', np.float32),
#                              ('z', np.float32),
#                              ('intensity', np.float32)]

#         try:
#             (trans_map, rot_map) = self.tf_listener.lookupTransform(self.frame_map, self.frame_lidar_sensor, header.stamp)
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#             print(e) #ForTesting
#             # self.get_logger().warning(f'No transform between {self.frame_map} and '
#             #               f'{self.frame_lidar_sensor} available with exception: {e}')
#             points = np.zeros((0, 3))
#             colours = np.zeros((0,))
#             trans_map = np.zeros(3)

#         # Apply the transformation to the point cloud data
#         rotation_matrix = tf_transformations.quaternion_matrix(rot_map)[:3, :3]
#         rotated_points = np.dot(points - trans_map, rotation_matrix.T)

#         pointcloud_data = np.zeros(rotated_points.shape[0], dtype=pointcloud_fields)
#         pointcloud_data['x'] = rotated_points[:, 0]
#         pointcloud_data['y'] = rotated_points[:, 1]
#         pointcloud_data['z'] = rotated_points[:, 2]
#         pointcloud_data['intensity'] = np.array(colours)
#         msg = ros2_numpy.msgify(PointCloud2, pointcloud_data)
#         msg.header = header
#         return msg

class LidarPublisher(SensorDataPublisher):
    def __init__(self, sensor, topic_id, vehicle):
        super().__init__(sensor, topic_id, PointCloud2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensor_name = topic_id.split("/")[-1]
        self.frame_lidar_sensor = f'{vehicle.vid}_{sensor_name}'
        self.points = np.zeros((0, 3)) #ForTesting
        self.colours = np.zeros((0,)) #ForTesting
        self.translation = np.zeros((0,)) #ForTesting

    def _make_msg(self):
        header = Header()
        header.frame_id = self.frame_lidar_sensor
        header.stamp = self.current_time
        # header.stamp = rclpy.clock.Clock().now().to_msg()  # type: ignore
        readings_data = self._sensor.poll()
        self.points = np.array(readings_data['pointCloud'])
        self.colours = readings_data['colours']

        pointcloud_fields = [('x', np.float32),
                             ('y', np.float32),
                             ('z', np.float32),
                             ('intensity', np.float32)]

        try:
            transform = self.tf_buffer.lookup_transform(self.frame_map, self.frame_lidar_sensor, header.stamp)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        except:
            # self.get_logger().warn(f'No transform between {self.frame_map} and ' f'{self.frame_lidar_sensor} available with exception: {e}') #ForTesting
            print(f'No transform between {self.frame_map} and {self.frame_lidar_sensor} available')
            self.points = np.zeros((0, 3))
            self.colours = np.zeros((0,))
            self.transform =  self.fromTranslationRotation([0, 0, 0], [0, 0, 0, 1])  # Identity transform
            # transform = tf2_ros.TransformerROS().fromTranslationRotation([0, 0, 0], [0, 0, 0, 1])  # Identity transform

        # Apply the transformation to the point cloud data
        translation = ros2_numpy.numpify(self.transform.transform.translation)
        rotation = ros2_numpy.numpify(self.transform.transform.rotation)
        rotation_matrix = self.fromTranslationRotation([0, 0, 0], rotation)[:3, :3]
        # rotation_matrix = tf2_ros.TransformerROS().fromTranslationRotation([0, 0, 0], rotation)[:3, :3]
        rotated_points = np.dot(self.points - self.transform , rotation_matrix.T)

        pointcloud_data = np.zeros(rotated_points.shape[0], dtype=pointcloud_fields)
        pointcloud_data['x'] = rotated_points[:, 0]
        pointcloud_data['y'] = rotated_points[:, 1]
        pointcloud_data['z'] = rotated_points[:, 2]
        pointcloud_data['intensity'] = np.array(self.colours)
        msg = ros2_numpy.msgify(PointCloud2, pointcloud_data)
        msg.header = header
        return msg

    def fromTranslationRotation(self, translation, rotation):
        return np.dot(self.translation_matrix(translation), self.quaternion_matrix(rotation))

    def translation_matrix(self, direction):
        M = np.identity(4)
        M[:3, 3] = direction[:3]
        return M

    def quaternion_matrix(self, quaternion):
        epsilon = np.finfo(float).eps * 4.0
        q = np.array(quaternion[:4], dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < epsilon:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)



    # def _make_msg(self):
    #     header = Header()
    #     header.frame_id = self.frame_lidar_sensor
    #     header.stamp = rclpy.clock.Clock().now().to_msg()  # type: ignore
    #     # header.stamp = self.get_clock().now().to_msg()

    #     readings_data = self._sensor.poll()
    #     points = np.array(readings_data['pointCloud'])
    #     colours = readings_data['colours']

    #     try:
    #         transform_stamped = self.tf_buffer.lookup_transform(
    #             self.frame_map,
    #          self.frame_lidar_sensor, header.stamp)
    #     except Exception as e:
    #         print(f'Transform exception: {e}') #ForTesting
    #         # self.get_logger().warning(f'Transform exception: {e}')
    #         return

    #     transformed_cloud = do_transform_cloud(pc2.create_cloud_xyz32(header, points), transform_stamped)
        

    #     msg = PointCloud2()
    #     msg.header = header
    #     msg.height = 1
    #     msg.width = len(transformed_cloud)
    #     msg.fields = [
    #         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    #     ]
    #     msg.is_bigendian = False
    #     msg.point_step = 16
    #     msg.row_step = msg.point_step * len(transformed_cloud)
    #     msg.is_dense = True
    #     msg.data = np.asarray(transformed_cloud).tobytes()

    #     return msg


    # def _make_msg(self):
    #     header = Header()
    #     header.frame_id = self.frame_lidar_sensor
    #     header.stamp = rclpy.clock.Clock().now().to_msg()  # type: ignore

    #     readings_data = self._sensor.poll()
    #     points = np.array(readings_data['pointCloud'])
    #     colours = readings_data['colours']

    #     try:
    #         transform_stamped = self.tf_buffer.lookup_transform(
    #             self.frame_map,
    #             self.frame_lidar_sensor,
    #             header.stamp)
    #     except Exception as e:
    #         self.get_logger().warning(f'Transform exception: {e}')
    #         return

    #     transformed_cloud = do_transform_cloud(pc2.create_cloud_xyz32(header, points), transform_stamped)

    #     msg = PointCloud2()
    #     msg.header = header
    #     msg.height = 1
    #     msg.width = len(transformed_cloud)
    #     msg.fields = [
    #         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    #     ]
    #     msg.is_bigendian = False
    #     msg.point_step = 16
    #     msg.row_step = msg.point_step * len(transformed_cloud)
    #     msg.is_dense = True
    #     msg.data = np.asarray(transformed_cloud).tobytes()

    #     return msg



    # def publish_data(self):
    #     msg = self._make_msg()
    #     self.publisher.publish(msg)

class VehiclePublisher(BNGPublisher):
# class VehiclePublisher(Node, BNGPublisher): #WIP
    def __init__(self, vehicle, node_name,visualize=True): #ForTesting
        BNGPublisher.__init__(self)
        # super().__init__(self, Node)

        
        
        self._vehicle = vehicle
        self.node_name = node_name
        self._sensor_publishers = list()
        # self.node = rclpy.create_node(node_name)
        # qos_profile = qos_profile_sensor_data(sensor_data=True)
        # self._broadcaster_pose = tf2_ros.TransformBroadcaster()
        # self._broadcaster_pose = tf2_ros.TransformBroadcaster(node_name,qos=qos_profile_sensor_data )
        self.tf_msg = TransformStamped()
        self.frame_map = 'map'
        self.tf_msg.header.frame_id = self.frame_map
        self.tf_msg.child_frame_id = self._vehicle.vid
        self.alignment_quat = np.array([0, 1, 0, 0])  # sets the forward direction as -y


        self.current_time = rclpy.clock.Clock().now().to_msg() # type: ignore
        # self.current_time = Clock().now()#ForTesting

        for sensor_name, sensor in vehicle.sensors.items():
            topic_id = [node_name, vehicle.vid, sensor_name]
            topic_id = '/'.join([str(x) for x in topic_id])
            print("------------")
            print("------------")
            print("topic_id in sensors's loop",topic_id)
            pub = get_sensor_publisher(sensor)
            print(f'pub: {pub}')  # For Testing
            if pub == CameraPublisher: #WIP 
                pub = pub(sensor, topic_id, self._vehicle)
            else:
                pub = pub(sensor, topic_id)
            self._sensor_publishers.append(pub)
            print("pub",pub)

        self.visualizer = None
        if visualize:
            topic_id = [node_name, vehicle.vid, 'marker']
            topic_id = '/'.join(topic_id)
            print("------------")
            print("------------")
            print("topic_id if visualizer",topic_id)
            # self._pub = self.create_publisher(MarkerArray, topic_id, 1)
            # self.visualizer = self.create_publisher(Marker, topic_id, qos_profile=qos_profile_sensor_data) #ForTesting
            self.visualizer = self.create_publisher(topic_id, Marker,  1) #WIP

    def broadcast_vehicle_pose(self, data):
        # self.tf_msg.header.stamp = self.current_time #ForTesting
        self.tf_msg.header.stamp = rclpy.clock.Clock().now().to_msg() # type: ignore
        self.tf_msg.transform.translation.x = data['pos'][0]
        self.tf_msg.transform.translation.y = data['pos'][1]
        self.tf_msg.transform.translation.z = data['pos'][2]
        quat_orientation = np.array([data['rotation'][0],
                                     data['rotation'][1],
                                     data['rotation'][2],
                                     data['rotation'][3]])

        quat_orientation /= np.linalg.norm(quat_orientation)

        self.tf_msg.transform.rotation.x = quat_orientation[0]
        self.tf_msg.transform.rotation.y = quat_orientation[1]
        self.tf_msg.transform.rotation.z = quat_orientation[2]
        self.tf_msg.transform.rotation.w = quat_orientation[3]

        # self._broadcaster_pose.sendTransform(self.tf_msg) #WIP
        self._broadcaster_pose.sendTransform(self.tf_msg, "/tf", qos=qos_profile_sensor_data ) #WIP

    def state_to_marker(self, data, marker_ns):
        mark = Marker()
        mark.header.frame_id = self.frame_map
        mark.header.stamp = self.current_time #ForTesting
        # mark.header.stamp = rclpy.clock.Clock().now().to_msg() # type: ignore
        mark.type = Marker.CUBE
        mark.ns = marker_ns
        mark.action = Marker.ADD
        mark.id = 0
        # mark.lifetime = Duration()
        # Set a long duration (e.g., 100 days)
        # seconds_per_day = 24 * 60 * 60  # 1 day = 24 hours * 60 minutes * 60 seconds
        # long_duration_seconds = 100 * seconds_per_day
        # long_duration = builtin_interfaces.msg.Duration(sec=long_duration_seconds, nanosec=0)
        # mark.lifetime = long_duration
        # mark.lifetime = builtin_interfaces.msg.Duration()  
        
        mark.pose.position.x = data['pos'][0]
        mark.pose.position.y = data['pos'][1]
        mark.pose.position.z = data['pos'][2]
        mark.pose.orientation.x = data['rotation'][0]
        mark.pose.orientation.y = data['rotation'][1]
        mark.pose.orientation.z = data['rotation'][2]
        mark.pose.orientation.w = data['rotation'][3]

        mark.scale.x = 5
        mark.scale.y = 1.9
        mark.scale.z = 1.5

        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.color.a = 1.0

        return mark



    def publish(self, current_time):
        self.current_time = current_time
        self._vehicle.poll_sensors()
        self.broadcast_vehicle_pose(self._vehicle.sensors['state'].data)
        for pub in self._sensor_publishers:
            threading.Thread(target=pub.publish, args=(current_time,), daemon=True).start()
        if self.visualizer is not None:
            mark = self.state_to_marker(self._vehicle.sensors['state'].data, self._vehicle.vid)
            self.visualizer.publish(mark)


class NetworkPublisher(BNGPublisher): #partially works, i get the num of makrkers, no markeres yet in rviz , same as ROS1 
# class NetworkPublisher(Node, BNGPublisher):
    def __init__(self, game_client, node_name):
        # super().__init__(self, Node)
        # Node.__init__(self, node_name)
        BNGPublisher.__init__(self) 
        
        self.frame_map = 'map'
        self._game_client = game_client
        self._road_network = None
        self._node_name = node_name
        topic_id = '/'.join([node_name, 'road_network'])
        # self._pub = self.create_publisher(MarkerArray, topic_id, 1) #WIP
        # self._pub = None
        
        # self.current_time = Clock().now()#ForTesting
        # self.current_time = Clock().now()#ForTesting
        self.current_time =  rclpy.clock.Clock().now().to_msg() # type: ignore

    def set_up_road_network_viz(self):
        roads = self._game_client.get_roads()
        network_def = dict()
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                network_def[int(r_id)] = self._game_client.get_road_edges(r_id)

        self._road_network = MarkerArray()
        for r_id, road in network_def.items():
            mark = Marker()
            mark.header = std_msgs.msg.Header()
            mark.header.frame_id = self.frame_map
            mark.header.stamp = self.current_time
            # print("currect time sec: ", self.current_time.sec)
            mark.type = Marker.LINE_STRIP
            ns = self._node_name
            mark.ns = ns
            mark.action = Marker.ADD
            mark.id = r_id
            
            # Set a long duration (e.g., 100 days)
            # seconds_per_day = 24 * 60 * 60  # 1 day = 24 hours * 60 minutes * 60 seconds
            # long_duration_seconds = 100 * seconds_per_day
            # long_duration = builtin_interfaces.msg.Duration(sec=long_duration_seconds, nanosec=0)
            # mark.lifetime=long_duration  
            
            # Clock().now() #ForTesting builtin_interfaces.msg.Time
            # long_duration=(self.current_time.sec*10)
            # mark.lifetime = builtin_interfaces.msg.Duration(long_duration)  # leave them up forever  
            # mark.lifetime = builtin_interfaces.msg.Duration(sec=0)  # leave them up forever  
            # mark.lifetime = Duration(seconds=0)  # leave them up forever  
                      
            mark.pose.position.x = 0.0
            mark.pose.position.y = 0.0
            mark.pose.position.z = 0.0
            mark.pose.orientation.x = 0.0
            mark.pose.orientation.y = 0.0
            mark.pose.orientation.z = 0.0
            mark.pose.orientation.w = 1.0

            mark.scale.x = 2.0
            mark.scale.y = 1.0

            mark.color.r = 1.0
            mark.color.b = 0.0
            mark.color.g = 0.0
            mark.color.a = 1.0
            for r_point in road:
                r_point = r_point['middle']
                p = geom_msgs.Point()
                p.x = r_point[0]
                p.y = r_point[1]
                p.z = r_point[2]
                mark.points.append(p) # type: ignore
            self._road_network.markers.append(mark) # type: ignore
        marker_num = len(self._road_network.markers)
        print(f'the road network contains {marker_num} markers')
        # self.get_logger().debug(f'the road network contains {marker_num} markers') #ForTesting

    def publish(self, current_time):
        self.current_time = current_time
        if self._road_network is None:
            self.set_up_road_network_viz()
        # self._pub.publish(self._road_network.markers) #WIP

