import math

## Получение yaw из текущей ориентации
def get_yaw_angle(self, msg):
    orientation = msg.pose.orientation
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    self.yaw_angle = math.atan2(siny_cosp, cosy_cosp)