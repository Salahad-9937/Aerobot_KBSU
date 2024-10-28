import cv2
import numpy as np
from sensor_msgs.msg import Image

class QRCodeHandler:
    def __init__(self, node):
        self.node = node
        self.qr_detector = cv2.QRCodeDetector()
        
        # Состояния для сканирования и переменных QR-кодов
        self.qr_detected = False
        self.bottom_qr_data = None
        self.front_qr_data = None
        self.search_bottom_qr = True  # Сначала ищем нижний QR-код

        # Подписка на изображение с нижней камеры
        self.bottom_image_sub = node.create_subscription(
            Image,
            '/uav1/camera_down',
            self.bottom_image_callback,
            10
        )

        # Подписка на изображение с передней камеры
        self.front_image_sub = node.create_subscription(
            Image,
            '/uav1/camera',
            self.front_image_callback,
            10
        )

    def bottom_image_callback(self, msg):
        if not self.search_bottom_qr:
            return

        image_data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        cv_image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

        data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)
        cv2.imshow("Bottom Camera", cv_image)
        cv2.waitKey(1)

        if data:
            self.bottom_qr_data = data
            self.qr_detected = True
            self.search_bottom_qr = False
            self.node.get_logger().info(f'Обнаружен QR-код на полу: {self.bottom_qr_data}')

    def front_image_callback(self, msg):
        if self.bottom_qr_data is None or self.front_qr_data is not None:
            return

        image_data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        cv_image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

        data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)
        cv2.imshow("Front Camera", cv_image)
        cv2.waitKey(1)

        if data:
            self.front_qr_data = data
            self.node.get_logger().info(f'Обнаружен QR-код на стене: {self.front_qr_data}')
            if self.bottom_qr_data == self.front_qr_data:
                self.node.get_logger().info("QR-коды совпадают, дрон летит вперёд")
            else:
                self.node.get_logger().info("QR-коды не совпадают, дрон продолжает искать")
