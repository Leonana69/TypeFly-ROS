import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_key

class KeyboardControlPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_control_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # Set up a timer to publish Twist messages at a regular interval
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.get_logger().info('Keyboard control node started. Press "w" for forward and "s" for backward.')

    def publish_twist(self):
        # Reset linear.x to 0.0 by default
        self.twist.linear.x = 0.0

        # Poll keyboard events
        events = get_key()
        print('events:', events)
        for event in events:
            if event.ev_type == 'Key' and event.state == 1:  # Key pressed
                if event.code == 'KEY_W':
                    self.twist.linear.x = 0.2  # Forward
                elif event.code == 'KEY_S':
                    self.twist.linear.x = -0.2  # Backward

        print(f'linear.x = {self.twist.linear.x}')
        # Publish the Twist message
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'Publishing: linear.x = {self.twist.linear.x}')

def main():
    rclpy.init()
    node = KeyboardControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
