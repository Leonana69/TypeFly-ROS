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
        self.timer = self.create_timer(0.01, self.publish_twist)
        self.get_logger().info('Keyboard control node started. Press "w" for forward and "s" for backward.')

        self.command_interval = 0.2
        self.last_command_time = self.get_clock().now()

    def publish_twist(self):
        # Poll keyboard events
        move_speed = 0.5
        turn_speed = 1.0
        is_key_release = False
        events = get_key()
        for event in events:
            if event.ev_type == 'Key' and (event.state == 1 or event.state == 2):  # Key pressed
                if event.code == 'KEY_W':
                    self.twist.linear.x = move_speed  # Forward
                elif event.code == 'KEY_S':
                    self.twist.linear.x = -move_speed  # Backward
                elif event.code == 'KEY_A':
                    self.twist.linear.y = move_speed
                elif event.code == 'KEY_D':
                    self.twist.linear.y = -move_speed
                elif event.code == 'KEY_E':
                    self.twist.angular.z = -turn_speed
                elif event.code == 'KEY_Q':
                    self.twist.angular.z = turn_speed
            elif event.ev_type == 'Key' and event.state == 0:  # Key released
                is_key_release = True
                if event.code == 'KEY_W' or event.code == 'KEY_S':
                    self.twist.linear.x = 0.0
                elif event.code == 'KEY_A' or event.code == 'KEY_D':
                    self.twist.linear.y = 0.0
                elif event.code == 'KEY_E' or event.code == 'KEY_Q':
                    self.twist.angular.z = 0.0

        # Publish the Twist message
        if is_key_release or (self.get_clock().now() - self.last_command_time).nanoseconds > self.command_interval * 1e9:
            self.publisher_.publish(self.twist)
            self.last_command_time = self.get_clock().now()
            self.get_logger().info(f'Publishing: {self.twist.linear.x} m/s, {self.twist.linear.y} m/s')

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
