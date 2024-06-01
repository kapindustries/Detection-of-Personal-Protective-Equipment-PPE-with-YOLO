import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_topic', 
            self.callback,
            10)
        self.publisher = self.create_publisher(Image, 'processed_video_topic', 10)  # Nuevo topic para la imagen procesada
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        try:
            # Convierte el mensaje de Image a un frame de OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error('Error al convertir mensaje de imagen: %s' % str(e))
            return

        # Mejorar el contraste del frame
        cv_image_contrast = cv2.convertScaleAbs(cv_image, alpha=1.5, beta=0)  

        # Reducir el brillo general
        cv_image_corrected = cv2.cvtColor(cv_image_contrast, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(cv_image_corrected)
        l_channel = cv2.subtract(l_channel, 30)  # Restar un valor fijo al canal de luminancia (L) para reducir el brillo general
        cv_image_corrected = cv2.merge((l_channel, a_channel, b_channel))

        # Reducir la luz blanca (ajustando los canales a y b)
        a_channel = cv2.subtract(a_channel, 0)  # Restar un valor fijo al canal a (ajustar según sea necesario)
        b_channel = cv2.subtract(b_channel, 0)  # Restar un valor fijo al canal b (ajustar según sea necesario)
        cv_image_corrected = cv2.merge((l_channel, a_channel, b_channel))

        cv_image_corrected = cv2.cvtColor(cv_image_corrected, cv2.COLOR_LAB2BGR)

        # Publicar la imagen procesada en el nuevo topic
        processed_msg = self.cv_bridge.cv2_to_imgmsg(cv_image_corrected)
        self.publisher.publish(processed_msg)

        # Muestra el video en una ventana
        cv2.imshow('Video con Contraste Mejorado, Brillo y Luz Blanca Reducidos', cv_image_corrected)
        cv2.waitKey(1)  # Espera un milisegundo

def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


