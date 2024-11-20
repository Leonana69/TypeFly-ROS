import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from flask import Flask, Response
import threading

from .utils import image_to_numpy

app = Flask(__name__)
image_frame = None

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Image):
        global image_frame
        numpy_image = image_to_numpy(msg)
        _, jpeg = cv2.imencode('.jpg', numpy_image)
        image_frame = jpeg.tobytes()
        # print('msg:', msg.width, msg.height, msg.encoding)

# Flask route to display the image
@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if image_frame is not None:
                # Serve the current image frame
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + image_frame + b'\r\n\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros_thread():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    # Start ROS 2 subscriber in a separate thread
    threading.Thread(target=ros_thread).start()
    # Start Flask server
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
