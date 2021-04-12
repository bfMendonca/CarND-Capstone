
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, int_windup_max, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.int_windup_max = int_windup_max

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time;

        if( integral > self.int_windup_max ):
            integral = self.int_windup_max
        elif( integral < -1.0*self.int_windup_max ):
            integral = -self.int_windup_max

        derivative = (error - self.last_error) / sample_time;

        val = self.kp * error + self.ki * integral + self.kd * derivative;

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
