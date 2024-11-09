from mavros_msgs.msg import ActuatorControl

## Поддержание тяги в полёте
def maintain_altitude(self):
    actuator_msg = ActuatorControl()
    actuator_msg.controls[3] = self.thrust  # Поддержание текущей тяги
    self.thrust_publisher.publish(actuator_msg)
    self.get_logger().info("Поддержание текущей высоты")