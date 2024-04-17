import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray  # Assuming this type for grid sensor data

class SensorAdapterNode(Node):
    def __init__(self):
        super().__init__('sensor_adapter_node')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'grid_sensor_topic', self.sensor_callback, 10)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)

    def sensor_callback(self, msg):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'sensor_frame'
        scan.angle_min = -1.5708  # -90 degrees
        scan.angle_max = 1.5708   # 90 degrees
        scan.angle_increment = 0.2618  # 15 degrees, adjust as per your sensor geometry
        scan.range_min = 0.2      # minimum range value
        scan.range_max = 10.0     # maximum range value
        scan.ranges = msg.data    # your distance data

        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = SensorAdapterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()