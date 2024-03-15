import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
              '/oak/points',
           # '/camera/depth/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process each point in the pointcloud
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[:3]  # Get the XYZ coordinates of the point
            if z < 2.0:  # Check if the point is within 2 meters in front of the camera
                self.get_logger().info(f'Obstacle detected at {x:.2f}, {y:.2f}, {z:.2f} meters')
                break  # Exit after first obstacle detection to reduce console spam

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
