from geometry_msgs.msg import Quaternion
import math

## Преобразование рысканья в кватернион
def yaw_to_quaternion(self, yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q