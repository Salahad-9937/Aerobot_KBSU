import math

## Установка текущего угла
def set_goal_angle(self, degree):
    initial_yaw = math.radians(degree)
    q = self.yaw_to_quaternion(initial_yaw)
    self.point.pose.orientation.x = q.x
    self.point.pose.orientation.y = q.y
    self.point.pose.orientation.z = q.z
    self.point.pose.orientation.w = q.w