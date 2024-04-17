import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
from std_msgs.msg import String

class PodController(Node):
    def __init__(self):
        super().__init__('pod_controller')
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 'pod_control', 10)
        pygame.init()
        self.screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption("Pod Controller")
        self.timer = self.create_timer(0.1, self.publish)
        self.control_msg = Float32MultiArray()
        self.control_msg.data = [0.0, 0.0, 0.0]


    def publish(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                key_name = pygame.key.name(event.key)
                self.get_logger().info(f"Press: {key_name}")
                if key_name == 'w':
                    self.control_msg.data[0] = 6
                elif key_name == 's':
                    self.control_msg.data[0] = -6
                elif key_name == 'a':
                    self.control_msg.data[1] = -6
                elif key_name == 'd':
                    self.control_msg.data[1] = 6
                elif key_name == 'q':
                    self.control_msg.data[2] = 10
                elif key_name == 'e':
                    self.control_msg.data[2] = -10
            elif event.type == pygame.KEYUP:
                key_name = pygame.key.name(event.key)
                self.get_logger().info(f"Release: {key_name}")
                if key_name == 'w':
                    self.control_msg.data[0] = 0
                elif key_name == 's':
                    self.control_msg.data[0] = 0
                elif key_name == 'a':
                    self.control_msg.data[1] = 0
                elif key_name == 'd':
                    self.control_msg.data[1] = 0
                elif key_name == 'q':
                    self.control_msg.data[2] = 0
                elif key_name == 'e':
                    self.control_msg.data[2] = 0
            elif event.type == pygame.QUIT:
                pygame.quit()
        self.control_publisher.publish(self.control_msg)

def main(args=None):
    rclpy.init(args=args)
    pod_controller = PodController()
    rclpy.spin(pod_controller)
    pod_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()