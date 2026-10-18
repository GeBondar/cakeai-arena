"""1:1 port of include/PIDController.h (CakeAI-V1) with an injected clock.

The original takes dt from std::chrono::steady_clock truncated to milliseconds;
here dt comes from clock() (seconds) with the same truncation, so the simulated
robot behaves like the real one at the same frame rate.
"""
import math


class PIDController:
    def __init__(self, clock, kp=1.0, ki=0.0, kd=0.0,
                 out_min=-math.inf, out_max=math.inf):
        self.clock = clock
        self.Kp, self.Ki, self.Kd = kp, ki, kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_min, self.output_max = out_min, out_max
        self.last_time = clock()
        self.first_run = True

    def calculate(self, target_angle, current_angle):
        error = target_angle - current_angle
        now = self.clock()
        dt = 0.01
        if not self.first_run:
            dt = int((now - self.last_time) * 1000) / 1000.0   # duration_cast<ms>
            if dt <= 0:
                dt = 0.001
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = 0.0
        if dt > 0 and not self.first_run:
            D = self.Kd * (error - self.previous_error) / dt
        output = P + I + D
        if output > self.output_max:
            output = self.output_max
            if error * output > 0:
                self.integral -= error * dt
        elif output < self.output_min:
            output = self.output_min
            if error * output > 0:
                self.integral -= error * dt
        self.previous_error = error
        self.last_time = now
        self.first_run = False
        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True
        self.last_time = self.clock()

    def set_parameters(self, kp, ki, kd):
        self.Kp, self.Ki, self.Kd = kp, ki, kd
        self.reset()

    def get_parameters(self):
        return self.Kp, self.Ki, self.Kd
