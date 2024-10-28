import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import ActuatorControl
from qr_code_module import QRCodeHandler
from drone_movement_module import DroneMovement

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard')

        # Настройка публикаторов и сервисов MAVROS
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.arming_s = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        
        # Публикация для управления скоростью двигателей
        self.actuator_control_pub = self.create_publisher(ActuatorControl, "/mavros/setpoint_raw/attitude", 10)

        # Инициализация начальной позиции и состояния
        self.point = PoseStamped()
        self.point.pose.position.x = 0.0
        self.point.pose.position.y = 0.0
        self.point.pose.position.z = 2.3  # Высота взлета

        # Инициализация модулей
        self.qr_handler = QRCodeHandler(self)    # Модуль для работы с QR-кодами
        self.movement = DroneMovement(self, self.point)  # Модуль для управления движением

        # Публикация начальной позиции для установки режима полёта
        for _ in range(10):
            self.setpoint_pub.publish(self.point)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Установка режима и взвод моторов
        self.set_mode_call("OFFBOARD")
        self.arming_call(True)

        # Таймер для публикации сетпоинтов
        self.timer = self.create_timer(0.1, self.publish_setpoint)

        # Подключение подписчиков на изображения и запуск движения
        # self.qr_handler.start_qr_detection()
        self.qr_handler.__init__
        self.movement.move_forward()

    def set_mode_call(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.set_mode.call_async(req)

    def arming_call(self, arm):
        req = CommandBool.Request()
        req.value = arm
        self.arming_s.call_async(req)

    def publish_setpoint(self):
        self.setpoint_pub.publish(self.point)
