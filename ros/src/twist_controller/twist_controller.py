import rospy
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.throttle_pid = PID( 1e-3, 0.0, 1e-9, 0.3, -10.0, +1.0)

                                        #(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
        self.yaw_controller = YawController( wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle )

        self.time = rospy.get_time()
        self.accel_value = 0.0
        self.brake_value = 0.0
        self.sterring_value = 0.0


        self.DECELERATION_CONSTANT = 0.1
        

    def clamp_value( self, value, max, min ):
    	if value > max:
    		return max
    	elif value < min:
    		return min
    	return value

    def control(self, twist_cmd, current_vel, enabled  ):
       	desired_speed = twist_cmd.twist.linear.x
       	current_speed = current_vel.twist.linear.x
        error = desired_speed - current_speed


       	now = rospy.get_time()
       	dt = (now - self.time)
       	self.time = now

        if( error > -abs(self.DECELERATION_CONSTANT*desired_speed) ) :
            error *= 1.2
        else:
            error *= 3000.0


        rospy.logwarn( 'error' + str( error ))


        output = self.throttle_pid.step( error, dt  )
        # desired_speed = current_velocity.twist
        # current_speed = 
        # error = current_speed - desired_speed

        if abs(desired_speed) > 0.1 and error > -abs(self.DECELERATION_CONSTANT*desired_speed) :
        	self.accel_value = self.clamp_value( self.accel_value + output, 0.5, 0 )
        	self.brake_value = 0.0
        elif abs(desired_speed) < 0.1:
            self.brake_value = 1300.0
            self.accel_value = 0.0
        else:
        	self.brake_value = self.clamp_value( self.brake_value + abs(output), 1300.0, 0 )
        	self.accel_value = 0.0

        #self.brake_value = self.clamp_value( self.brake_value - output, 100.0, 0 )
        steering = self.yaw_controller.get_steering( desired_speed, twist_cmd.twist.angular.z, current_speed )

        return self.accel_value, self.brake_value, steering
