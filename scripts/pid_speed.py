from ublox_msgs.msg import NavPVT

class PIDController:
    def __init__(self, setpoint):
        self.Kp = .5
        self.Ki = .1
        self.Kd = .2
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, process_variable):
        error = self.setpoint - process_variable
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output