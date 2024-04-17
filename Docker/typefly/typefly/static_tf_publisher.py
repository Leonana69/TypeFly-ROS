import rclpy
import tf2_ros
import geometry_msgs.msg
from rclpy.node import Node

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def publish_static_tf(self):
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'base_scan'
        transform_stamped.transform.translation.x = 0.0  # Adjust these values as needed
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform([transform_stamped])
