import math
from increase_thrust import increase_thrust
from maintain_altitude import maintain_altitude

## Коллбэк позиции и ориентации
def pose_callback(self, msg):
    ## Получение yaw из текущей ориентации
    orientation = msg.pose.orientation
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    self.yaw_angle = math.atan2(siny_cosp, cosy_cosp)

    ## Получение высоты
    current_altitude = msg.pose.position.z
    # Проверка высоты и регулировка тяги при необходимости
    if current_altitude < self.target_altitude:
        increase_thrust(self)
    else:
        maintain_altitude(self)
