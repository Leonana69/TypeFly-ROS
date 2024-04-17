import rclpy
from rclpy.node import Node
from podtp import Podtp
from std_msgs.msg import Float32MultiArray

from .pod_sensor_data_publisher import PodSensorDataPublisher
from .static_tf_publisher import StaticTFPublisher

class PodMain(Node):
    def __init__(self):
        super().__init__('pod_main')
        self.podtp = Podtp({'ip': '192.168.0.115', 'port': 80, "stream_port": 81 })
        if self.podtp.connect() and self.podtp.send_ctrl_lock(False):
            self.get_logger().info("Pod Connection [OK]")
            self.podtp.reset_estimator()
        else:
            self.get_logger().info("Pod Connection [FAILED]")
            return

        self.timer = self.create_timer(0.1, self.publish)
        self.pod_sensor_data_publisher = PodSensorDataPublisher(self.podtp)
        self.static_tf_publisher = StaticTFPublisher()
        self.static_tf_publisher.publish_static_tf()

        self.control_sub = self.create_subscription(
            Float32MultiArray, 'pod_control', self.control_callback, 10)

    def control_callback(self, msg):
        self.podtp.send_command_hover(0, msg.data[0], msg.data[1], msg.data[2])

    def publish(self):
        self.pod_sensor_data_publisher.publish_sensor_data()

def main(args=None):
    rclpy.init(args=args)
    pod_ros = PodMain()
    try:
        while rclpy.ok():
            rclpy.spin_once(pod_ros)
    except KeyboardInterrupt:
        pass
    finally:
        pod_ros.destroy_node()
        static_tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()