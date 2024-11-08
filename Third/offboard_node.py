import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, SetMode
# from mavros_msgs.msg import ActuatorControl
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from set_mode_call import set_mode_call
from convert_360_to_180 import convert_360_to_180
from yaw_to_quaternion import yaw_to_quaternion
from set_goal_angle import set_goal_angle
from get_yaw_angle import get_yaw_angle

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard')
        # Создаём профиль QoS с настройками истории, глубины и надёжности
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Настройка публикаторов, подписчиков и сервисов MAVROS
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 20)
        self.arming_s = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        # self.actuator_control_pub = self.create_publisher(ActuatorControl, "/mavros/setpoint_raw/attitude", 10)
        self.local_position_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.get_yaw_angle, qos_profile)

        # Инициализация начальной позиции и состояния
        self.point = PoseStamped()
        self.point.pose.position.z = 3.5  # Высота взлета
        self.yaw_angle = 0.0  # Изначальное значение yaw
        self.step_distance = 0.05 # расстояние для медленного движения вперед # 0.05 оптимально
        # заданные точки движения
        self.current_dx = self.step_distance * math.cos(self.yaw_angle*180/3.14)
        self.current_dy = self.step_distance * math.sin(self.yaw_angle*180/3.14)
        # Хрень для движежения круга
        self.n = 90

        # Установка yaw на 90 градусов влево
        self.set_goal_angle(90)

        # Публикация начальной позиции для установки режима полёта
        for _ in range(10):
            self.setpoint_pub.publish(self.point)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Установка режима и взвод моторов, а также таймер публикации сетпоинтов
        self.set_mode_call("OFFBOARD", True)
        self.timer = self.create_timer(0.1, self.publish_setpoint)

        # Таймер для медленного движения вперед
        self.forward_timer = self.create_timer(0.25, self.move_forward_slowly)

    ### METHODS ###
    def yaw_to_quaternion(self, yaw):
        return yaw_to_quaternion(self, yaw)
    def set_mode_call(self, custom_mode, arm):
        return set_mode_call(self, custom_mode, arm)
    def publish_setpoint(self):
        self.setpoint_pub.publish(self.point)
    def convert_360_to_180(self, angle_360):
        return convert_360_to_180(self, angle_360)
    def set_goal_angle(self, degree):
        return set_goal_angle(self, degree)
    def get_yaw_angle(self, msg):
        return get_yaw_angle(self, msg)

    ## метод для движения
    def move_forward_slowly(self):
        yaw_degree_angle = self.yaw_angle*180/3.14
        
        ## Задание кругового движения
        self.n += 1
        if (self.n==45 or self.n==90 or self.n==135 or self.n==180 or self.n==225 or self.n==270 or self.n==315 or self.n==360):
            self.set_goal_angle(self.n)
            c_n = self.convert_360_to_180(self.n)
            print(c_n)
        if(self.n > 360):
            self.n = 0
        print(yaw_degree_angle)

        ## Проверка достижения угла (n - заданный угол)
        if (yaw_degree_angle <= self.convert_360_to_180(self.n)+0.1 or
            yaw_degree_angle >= self.convert_360_to_180(self.n)-0.1):
            dx = self.step_distance * math.cos(self.yaw_angle)
            dy = self.step_distance * math.sin(self.yaw_angle)
            ## Проверка достижения точки
            if (dx <= self.current_dx+0.02 or dx >= self.current_dx-0.02 and
                dy <= self.current_dy+0.02 or dy >= self.current_dy-0.02):
                ## Обновление текущих точек
                self.current_dx = dx
                self.current_dy = dy
                ## Обновление позиции с учётом ориентации
                self.point.pose.position.z = 2.5 # позиция в воздухе
                self.point.pose.position.x += dx
                self.point.pose.position.y += dy
                self.setpoint_pub.publish(self.point)
        self.point.pose.position.z = 2.5 # позиция в воздухе
        self.setpoint_pub.publish(self.point)
    


