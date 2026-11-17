import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter('device', "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0")
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fourcc', 'MJPG')

        device = self.get_parameter('device').value
        rate = self.get_parameter('rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fourcc = self.get_parameter('fourcc').value
        
        self.publisher_ = self.create_publisher(Image, 'video_frames', qos_profile_sensor_data)
        self.br = CvBridge()
        
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera')
            
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, rate)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.get_logger().info(
            f'Opened camera: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
            f'@ {self.cap.get(cv2.CAP_PROP_FPS):.1f} fps'
        )

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return
        frame = cv2.flip(frame, 0)
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
