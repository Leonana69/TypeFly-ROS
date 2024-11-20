from sensor_msgs.msg import Image
import numpy as np

def image_to_numpy(image: Image) -> np.ndarray:
    data = image.data
    width = image.width
    height = image.height
    encoding = image.encoding
    step = width * 3
    return np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))