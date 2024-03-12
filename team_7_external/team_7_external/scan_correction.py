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

import math
import sys

from std_msgs.msg import String

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ScanCorrection(Node):
    """
    Scan Correction helps define usable angle range of your "/scan" topic (or your lidar data topic)

    "/scan_corrected" will be publish by modifying "/scan" (or your lidar data topic) data.
    Use "scan_correction.yaml" to edit create certain usable ranges of angle.
    """

    def __init__(self):
        super().__init__('scan_correction')

        # declare params that can be passed by "scan_correction.yaml"
        self.declare_parameters(namespace='', 
                                parameters=[
                                    ('original_lidar_topic', '/scan'),
                                    ('topic_name', 'scan_corrected'),
                                    ('number_of_ranges', 0),
                                    ('ranges_min', []),
                                    ('ranges_max', []),
                                ])
        self.original_lidar_topic = self.get_parameter('original_lidar_topic').value
        self.topic_name = self.get_parameter('topic_name').value
        self.number_of_ranges = self.get_parameter('number_of_ranges').value
        self.ranges_min = self.get_parameter('ranges_min').value
        self.ranges_max = self.get_parameter('ranges_max').value

        # create a publisher and a subscriber
        self.scan_corrected_pubisher = self.create_publisher(LaserScan, 
                                                             self.topic_name, 
                                                             10)
        self.scan_subscriber = self.create_subscription(LaserScan,
                                                        self.original_lidar_topic, 
                                                        self.laser_callback, 
                                                        QoSProfile(depth=10, 
                                                                   reliability=ReliabilityPolicy.RELIABLE)
                                                        )

        # define the timer period for 0.02 seconds
        self.timer_period = 0.02
        
        ### Initialize Variables
        self.scan = LaserScan()

        self.scan_corrected = LaserScan()
        self.beam_size_ = 0.0

        # create a Timer
        self.timer = self.create_timer(self.timer_period, self.scan_correction_publish)

    def laser_callback(self,msg):
        # Save scan info
        self.scan = msg
    
    def scan_correction_publish(self):
        # Set Unchanged variables
        self.scan_corrected.header = self.scan.header

        self.scan_corrected.angle_min = self.scan.angle_min
        self.scan_corrected.angle_max = self.scan.angle_max

        self.scan_corrected.angle_increment = self.scan.angle_increment
        self.scan_corrected.time_increment = self.scan.time_increment
        self.scan_corrected.scan_time = self.scan.scan_time
        self.scan_corrected.range_min = self.scan.range_min
        self.scan_corrected.range_max = self.scan.range_max

        self.scan_corrected.ranges = self.scan.ranges
        self.scan_corrected.intensities = self.scan.intensities
        
        # Adjust Usable Data
        if self.scan_corrected.angle_increment != 0:
            self.beam_size_ = math.ceil((self.scan_corrected.angle_max - self.scan_corrected.angle_min) / self.scan_corrected.angle_increment)

            # self.get_logger().info('Publishing: "%f"' % self.beam_size_)
            for i in range(0,self.beam_size_):
                for j in range(0,self.number_of_ranges):
                    q1 = math.ceil((self.ranges_min[j] / 180 * math.pi - self.scan_corrected.angle_min) / self.scan_corrected.angle_increment)
                    q2 = math.ceil((self.ranges_max[j] / 180 * math.pi - self.scan_corrected.angle_min) / self.scan_corrected.angle_increment)
                    
                    if q1 <= i and i < q2:
                        self.scan_corrected.ranges[i] = float('inf')

        # Publishing the scan_corrected values to a topic
        self.scan_corrected_pubisher.publish(self.scan_corrected)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    scan_correction_node = ScanCorrection()
    try:
        rclpy.spin(scan_correction_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_correction_node.destroy_node()
    rclpy.shutdown()
