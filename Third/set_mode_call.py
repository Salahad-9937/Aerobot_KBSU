from mavros_msgs.srv import CommandBool, SetMode    

## Установка режима и арминг
def set_mode_call(self, custom_mode, arm):
    req1 = SetMode.Request()
    req1.custom_mode = custom_mode
    self.set_mode.call_async(req1)
    req2 = CommandBool.Request()
    req2.value = arm
    self.arming_s.call_async(req2)