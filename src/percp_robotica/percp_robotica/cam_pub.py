import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_topic', 10)
        self.cv_bridge = CvBridge()

        # Captura de video desde la cámara
        self.cap = cv2.VideoCapture(0)  
        if not self.cap.isOpened():
            self.get_logger().error("Error al abrir la cámara.")
            return

        self.timer = self.create_timer(1.0 / 30, self.publish_frame)  # Publica cada 1/30 segundo

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convierte el frame de OpenCV a un mensaje de imagen
            msg = self.cv_bridge.cv2_to_imgmsg(frame)
            # Publica el mensaje
            self.publisher_.publish(msg)
        else:
            # Cierra la cámara si no se puede leer más
            self.cap.release()
            self.get_logger().info("Captura de video finalizada.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
