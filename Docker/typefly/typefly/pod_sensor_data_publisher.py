import rclpy
from rclpy.time import Time, Duration
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf_transformations
import tf2_ros
import math
from geometry_msgs.msg import Quaternion, TransformStamped
import geometry_msgs.msg
from podtp import Podtp

from .static_tf_publisher import StaticTFPublisher

class PodSensorDataPublisher(Node):
    def __init__(self, podtp: Podtp):
        super().__init__('pod_sensor_data_publisher')
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.transform_publisher = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Pod sensor_data Publisher [START]")
        self.podtp = podtp

    def get_scan(self) -> LaserScan:
        # each contains 8 values
        depth_front = self.podtp.sensor_data.depth.data.astype(np.float)[3, :] / 1000
        depth_left = self.podtp.sensor_data.depth.data.astype(np.float)[6, :] / 1000
        depth_right = self.podtp.sensor_data.depth.data.astype(np.float)[7, :] / 1000
        # depth_front = np.random.rand(8, 8)[3, :]  # Random depth data between 0 and 10 meters
        # for i in range(8):
        #     depth_front[i] = 1.2
        scan = LaserScan()
        scan.header.stamp = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        scan.header.frame_id = 'base_scan'
        scan.angle_min = -1.914626  # -112.5 + 2.8 degrees
        scan.angle_max = 1.914626   # 112.5 - 2.8 degrees
        scan.angle_increment = 0.0982  # 5.625 degrees, 45 degrees total divided by 8 measurements
        scan.range_min = 0.05      # minimum range value
        scan.range_max = 5.0     # maximum range value
        scan.ranges = [float('nan')] * 40
        for i in range(8):
            if depth_front[i] < scan.range_min:
                depth_front[i] = float('nan')
            elif depth_front[i] > scan.range_max:
                depth_front[i] = float('nan')
            else:
                depth_front[i] = depth_front[i] / math.cos(-0.343829 + i * 0.0982)

            if depth_left[i] < scan.range_min:
                depth_left[i] = float('nan')
            elif depth_left[i] > scan.range_max:
                depth_left[i] = float('nan')
            else:
                depth_left[i] = depth_left[i] / math.cos(-0.343829 + i * 0.0982)        


            if depth_right[i] < scan.range_min:
                depth_right[i] = float('nan')
            elif depth_right[i] > scan.range_max:
                depth_right[i] = float('nan')
            else:
                depth_right[i] = depth_right[i] / math.cos(-0.343829 + i * 0.0982)

        for i in range(8):
            scan.ranges[i] = depth_right[7 - i]
            scan.ranges[16 + i] = depth_front[7 - i]
            scan.ranges[32 + i] = depth_left[7 - i]
        return scan

    def get_odom(self):
        state = self.podtp.sensor_data.state.data.astype(np.float)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = state[0] / 1000.0
        odom_msg.pose.pose.position.y = state[1] / 1000.0
        odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, state[5] / 1000)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        return odom_msg

    def get_tranform(self):
        state = self.podtp.sensor_data.state.data.astype(np.float)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = state[0] / 1000.0
        t.transform.translation.y = state[1] / 1000.0
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, state[5] / 1000)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return t

    def publish_sensor_data(self):
        if not self.podtp.connected:
            self.get_logger().info("Pod Not Connected")
            return
        self.scan_publisher.publish(self.get_scan())
        self.transform_publisher.sendTransform(self.get_tranform())
        self.odom_publisher.publish(self.get_odom())
        # self.get_logger().info('Publishing sensor data')
