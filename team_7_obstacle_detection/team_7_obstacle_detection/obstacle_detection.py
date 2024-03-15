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

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ObstacleDetector(Node):
    """
    Obstacle Detector is currently a simple detector using PointCloud2 providing avoidance ability

    "/error" will pid error data.
    Use "scan_correction.yaml" to edit create certain usable ranges of angle.
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # declare params that can be passed by "scan_correction.yaml"
        self.declare_parameters(namespace='', 
                                parameters=[
                                    ('original_lidar_topic', '/scan'),
                                    ('topic_name', 'error'),
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
        self.error_pubisher = self.create_publisher(Float32MultiArray, 
                                                    self.topic_name, 
                                                    10)

        self.pcl_subscriber = self.create_subscription(PointCloud2,
                                                       '/oak/points',
                                                       self.pcl_callback,
                                                       QoSProfile(depth=10, 
                                                                  reliability=ReliabilityPolicy.BEST_EFFORT)
                                                       )
        self.pcl_subscriber  # prevent unused variable warning

        #### Initialize Variables
        self.obstacle_found = False
        self.error_msg = Float32MultiArray() # [error_distance_to_wall, heading_error_with_wall]
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        # self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim[0].label = "lateral_error"
        self.error_msg.layout.dim[1].label = "longitdudinal_error"
        self.error_msg.layout.dim[2].label = "heading_error"
        # self.error_msg.layout.dim[3].label = "future_curvature"
        self.error_msg.layout.dim[0].size = 1
        self.error_msg.layout.dim[1].size = 1
        self.error_msg.layout.dim[2].size = 1
        # self.error_msg.layout.dim[3].size = 1

        # define the timer period for 0.02 seconds
        self.timer_period = 0.02

        # create a Timer
        self.timer = self.create_timer(self.timer_period, self.error_publish)

    def pcl_callback(self, msg):
        # Process each point in the pointcloud
        self.obstacle_found = False
        #pcl_as_numpy_array = np.array(list(msg.data))
        for point in pc2.read_points(msg):
            x, y, z = point[:3]  # Get the XYZ coordinates of the point
            if z < 2:  # Check if the point is within 2 meters in front of the camera
                self.obstacle_found = True
                self.get_logger().info(f'Obstacle detected at {x:.2f}, {y:.2f}, {z:.2f} meters')
                break  # Exit after first obstacle detection to reduce console spam

    def error_publish(self):
        lateral_error = 0
        longitdudinal_error = 0
        
        if self.obstacle_found:
            heading_error = 1
        else:
            heading_error = 0

        self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error)]
        self.error_pubisher.publish(self.error_msg)

def main(args=None):
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    obstacle_detector_node = ObstacleDetector()
    try:
        rclpy.spin(obstacle_detector_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
