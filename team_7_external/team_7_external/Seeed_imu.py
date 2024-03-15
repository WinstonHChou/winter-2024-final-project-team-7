# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import time

import sys
import os
from glob import glob

sys.path.insert(0, 'razorIMU_9dof')
from imu import *
from config_ref import *
from config_writer import ConfigWriter

import math

from std_msgs.msg import String

from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from tf_transformations import quaternion_from_euler

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

DEGREES_2_RAD = math.pi/180.0
MIN_YAW_CALIBRATION = -10
MAX_YAW_CALIBRATION = 10

config_path = "Seeed_imu.yaml"

# class SeeedImu(Node):
#     """
#     Seeed Imu supports the 6-Axis IMU Usage on Seeed Studio XIAO nRF52840 Sense

#     "/imu" for imu data.
#     "/diagnostics" for diagnostics.
#     """

#     def __init__(self):
#         super().__init__('seeed_imu')

#         # create a publisher and a subscriber
#         self.imu_pub_ = self.create_publisher(Imu, 'imu', rclpy.qos.qos_profile_sensor_data)
#         self.diag_pub_ = self.create_publisher(DiagnosticArray, 'diagnostics', rclpy.qos.qos_profile_sensor_data)

#         self.declare_parameter('imu_yaw_calibration', 0.0)
#         self.imu_yaw_calib_ = self.get_parameter('imu_yaw_calibration').value

#         self.declare_parameter('orientation_covariance', 
#             [0.0025 , 0.0 , 0.0,
#             0.0, 0.0025, 0.0,
#             0.0, 0.0, 0.0025]
#         )
#         self.orientation_covariance_ = self.get_parameter('orientation_covariance').value

#         self.declare_parameter('angular_velocity_covariance', 
#             [0.02, 0.0 , 0.0,
#             0.0 , 0.02, 0.0,
#             0.0 , 0.0 , 0.02]
#         )
#         self.angular_velocity_covariance_ = self.get_parameter('angular_velocity_covariance').value

#         self.declare_parameter('linear_acceleration_covariance',
#             [0.04 , 0.0 , 0.0,
#             0.0 , 0.04, 0.0,
#             0.0 , 0.0 , 0.04]
#         )
#         self.linear_acceleration_covariance_ = self.get_parameter('linear_acceleration_covariance').value

#         self.Razor_Imu_ = Razor_Imu(config_path)

#         self.port_ = self.Razor_Imu_.port

#         self.declare_parameter("frame_id", "base_imu_link")
#         self.frame_id_ = self.get_parameter('frame_id').value

#         # define the timer period for 0.02 seconds
#         self.timer_period = 0.02
        
#         ### Initialize Variables

#         # create a Timer
#         self.timer = self.create_timer(self.timer_period, self.imu_publish)

#     def imu_publish(self):
#         print("X: " + str(razor_imu.orient[AxisRef.X.value]))
#         print("Y: " + str(razor_imu.orient[AxisRef.Y.value]))
#         print("Z: " + str(razor_imu.orient[AxisRef.Z.value]))
#         time.sleep(1)


# def main():
#     logger = rclpy.logging.get_logger('logger')

#     # pass parameters and initialize node
#     rclpy.init()
#     seeed_imu_node = SeeedImu()
#     try:
#         rclpy.spin(seeed_imu_node)
#     except KeyboardInterrupt:
#         pass
    
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     seeed_imu_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

if __name__ == "__main__":
    razor_imu = RazorIMU(config_path)
    razor_imu.start()
    time.sleep(2)
    for t in range(1,20):
        print("X: " + str(razor_imu.orient[AxisRef.X.value]))
        print("Y: " + str(razor_imu.orient[AxisRef.Y.value]))
        print("Z: " + str(razor_imu.orient[AxisRef.Z.value]))
        time.sleep(1)
    razor_imu.shutdown()
    print("All Good!")