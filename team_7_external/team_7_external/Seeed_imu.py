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

sys.path.insert(0, '/home/projects/ros2_ws/src/winter-2024-final-project-team-7/team_7_external/team_7_external/razorIMU_9dof')
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

config_path = "/home/projects/ros2_ws/src/winter-2024-final-project-team-7/team_7_external/config/Seeed_imu.yaml"

def angular_modulus(deg):
    if deg > 180.0:
        deg = deg - 360
        return angular_modulus(deg)
    if deg < -180.0:
        deg = deg + 360
        return angular_modulus(deg)
    return deg

class SeeedImu(Node):
    """
    Seeed Imu supports the 6-Axis IMU Usage on Seeed Studio XIAO nRF52840 Sense

    "/imu" for imu data.
    "/diagnostics" for diagnostics (Not implemented).
    """

    def __init__(self):
        super().__init__('seeed_imu')

        # declare params that can be passed by "Seeed_imu_config.yaml"
        self.declare_parameters(namespace='', 
                                parameters=[
                                    ('TBD', 'TBD'),
                                    ('imu_Debug_output', False),
                                ])
        self.TBD = self.get_parameter('TBD').value
        self.imu_Debug_output = self.get_parameter('imu_Debug_output').value

        # create a publisher and a subscriber
        self.imu_pub_ = self.create_publisher(Imu, 'imu', rclpy.qos.qos_profile_sensor_data)
        # self.diag_pub_ = self.create_publisher(DiagnosticArray, 'diagnostics', rclpy.qos.qos_profile_sensor_data)

        self.declare_parameter('imu_yaw_calibration', 0.0)
        self.imu_yaw_calib_ = self.get_parameter('imu_yaw_calibration').value

        self.declare_parameter('orientation_covariance', 
            [0.0025 , 0.0 , 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025]
        )
        self.orientation_covariance_ = self.get_parameter('orientation_covariance').value

        self.declare_parameter('angular_velocity_covariance', 
            [0.02, 0.0 , 0.0,
            0.0 , 0.02, 0.0,
            0.0 , 0.0 , 0.02]
        )
        self.angular_velocity_covariance_ = self.get_parameter('angular_velocity_covariance').value

        self.declare_parameter('linear_acceleration_covariance',
            [0.04 , 0.0 , 0.0,
            0.0 , 0.04, 0.0,
            0.0 , 0.0 , 0.04]
        )
        self.linear_acceleration_covariance_ = self.get_parameter('linear_acceleration_covariance').value
        
        self.Razor_Imu_ = RazorIMU(config_path)
        self.Razor_Imu_.start()
        time.sleep(2)

        self.port_ = self.Razor_Imu_.port

        self.declare_parameter("frame_id", "base_imu_link")
        self.frame_id_ = self.get_parameter('frame_id').value

        self.declare_parameter("calibration_magn_use_extended", False)
        self.calibration_magn_use_extended_ = self.get_parameter('calibration_magn_use_extended').value
        
        self.declare_parameter("magn_ellipsoid_center", [0.0, 0.0, 0.0])
        self.magn_ellipsoid_center_ = self.get_parameter('magn_ellipsoid_center').value

        self.declare_parameter("magn_ellipsoid_transform",
            [0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0]
        )
        magn_ellipsoid_transform_1d = self.get_parameter('magn_ellipsoid_transform').value
        self.magn_ellipsoid_transform_ = np.zeros((len(self.magn_ellipsoid_center_), len(self.magn_ellipsoid_center_)))
        for i in range(len(self.magn_ellipsoid_center_)):
            for j in range(len(self.magn_ellipsoid_center_)):
                self.magn_ellipsoid_transform_[i][j] = magn_ellipsoid_transform_1d[i + j]

        # get_logger
        self.get_logger().info(f"Opening {self.port_}")
        self.declare_parameter("baudrate", 57600)

        ### Initialize Variables
        self.imuMsg = Imu()
        self.imuMsg.orientation_covariance = self.orientation_covariance_
        self.imuMsg.angular_velocity_covariance = self.angular_velocity_covariance_
        self.imuMsg.linear_acceleration_covariance = self.linear_acceleration_covariance_

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.roll = self.roll_deg * DEGREES_2_RAD
        self.pitch = self.pitch_deg * DEGREES_2_RAD
        self.yaw = self.yaw_deg * DEGREES_2_RAD

        # define the timer period for 0.02 seconds
        self.timer_period = 0.02

        # create a Timer
        self.timer = self.create_timer(self.timer_period, self.imu_publish)

    def imu_publish(self):

        self.roll_deg = angular_modulus(self.Razor_Imu_.orient[AxisRef.X.value])
        self.pitch_deg = angular_modulus(self.Razor_Imu_.orient[AxisRef.Y.value])
        self.yaw_deg = angular_modulus(self.Razor_Imu_.orient[AxisRef.Z.value])
        
        if self.imu_Debug_output:
            self.get_logger().info('Publishing: "Roll: %f, Pitch: %f, Yaw: %f"' % (self.roll_deg, self.pitch_deg, self.yaw_deg))

        self.roll = self.roll_deg * DEGREES_2_RAD
        self.pitch = self.pitch_deg * DEGREES_2_RAD
        self.yaw = self.yaw_deg * DEGREES_2_RAD

        self.imuMsg.linear_acceleration.x = self.Razor_Imu_.accel[AxisRef.X.value]
        self.imuMsg.linear_acceleration.y = self.Razor_Imu_.accel[AxisRef.Y.value]
        self.imuMsg.linear_acceleration.z = self.Razor_Imu_.accel[AxisRef.Z.value]

        self.imuMsg.angular_velocity.x = self.Razor_Imu_.gyro[AxisRef.X.value]
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        self.imuMsg.angular_velocity.y = self.Razor_Imu_.gyro[AxisRef.Y.value]
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        self.imuMsg.angular_velocity.z = self.Razor_Imu_.gyro[AxisRef.Z.value]

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.imuMsg.orientation.x = q[0]
        self.imuMsg.orientation.y = q[1]
        self.imuMsg.orientation.z = q[2]
        self.imuMsg.orientation.w = q[3]
        self.imuMsg.header.stamp= self.get_clock().now().to_msg()
        self.imuMsg.header.frame_id = self.frame_id_

        self.imu_pub_.publish(self.imuMsg)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    seeed_imu_node = SeeedImu()
    try:
        rclpy.spin(seeed_imu_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    seeed_imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# if __name__ == "__main__":
#     razor_imu = RazorIMU(config_path)
#     razor_imu.start()
#     time.sleep(2)
#     for t in range(1,20):
#         roll = angular_modulus(razor_imu.orient[AxisRef.X.value])
#         pitch = angular_modulus(razor_imu.orient[AxisRef.Y.value])
#         yaw = angular_modulus(razor_imu.orient[AxisRef.Z.value])
#         print("X: " + str(roll))
#         print("Y: " + str(pitch))
#         print("Z: " + str(yaw))
#         time.sleep(1)
#     razor_imu.shutdown()
#     print("All Good!")