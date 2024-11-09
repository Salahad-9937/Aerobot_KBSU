from mavros_msgs.msg import ActuatorControl

## Увеличение тяги для взлёта
def increase_thrust(self):
    actuator_msg = ActuatorControl()
    actuator_msg.controls[3] = self.thrust  # Установка тяги (ось Z)
    self.thrust += 0.05  # Увеличение тяги
    if self.thrust > 1.5:
        self.thrust = 1.5  # Максимальная тяга
    self.thrust_publisher.publish(actuator_msg)
    self.get_logger().info(f"Увеличение тяги до {self.thrust}")