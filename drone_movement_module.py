from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ActuatorControl

class DroneMovement:
    def __init__(self, node, point):
        self.node = node
        self.point = point

    def move_forward(self, distance=2.0):
        self.point.pose.position.x += distance
        self.point.pose.position.z = 2.3
        self.set_motor_speed(0.3)
        self.node.get_logger().info(f"Дрон движется на {distance} метра вперёд")

    def set_motor_speed(self, thrust=0.5):
        control_msg = ActuatorControl()
        control_msg.controls = [0.0, 0.0, thrust, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.node.actuator_control_pub.publish(control_msg)
