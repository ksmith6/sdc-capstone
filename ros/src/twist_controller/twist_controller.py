from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from rospy import logwarn

from math import atan, sin , cos

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

#Got this value from the slack channel
#Will remove hardcoded number at a later date if it is useless and replace with the parameter from ROS
BRAKE_MAX = 20000.0 

class Controller(object):

   def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.max_angle = max_steer_angle
	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
	#Currently using just a P controller for throttle control
        self.pid_throttle = PID(0.35,0.0,0.0,-1.0,1.0)
	#Currently using a PD controller for steering control
        #self.pid_steer = PID(0.3,0.0,0.03,-self.max_angle, self.max_angle)
	#Low pass filter smooths out the steering
	self.lowpass_steer = LowPassFilter(0.2,1.0)
	#self.lowpass_angle = LowPassFilter(1.2,1.0)

   def control(self,linear_velocity, angular_velocity, current_linear, pose, way_points, dbw_enabled, time_elapsed):
	#Reset if drive by wire is not enabled and return 0's
        if not dbw_enabled:
		self.pid_throttle = PID(0.35,0.0,0.0,-1.0,1.0)
		#self.pid_steer = PID(0.3,0.0,0.03,-self.max_angle, self.max_angle)
		self.lowpass_steer = LowPassFilter(0.2,1.0)
		#self.lowpass_angle = LowPassFilter(1.2,1.0)
		return 0.0, 0.0, 0.0

	#Calculate CTE for throttle
	linear_velocity = 4.47 # This will keep the car at 10 MPH. To remove once a controller can handle higher speeds
	cte = linear_velocity - current_linear 
	pid_throttle_output = self.pid_throttle.step(cte, time_elapsed)
	#Throttle and brake should not be used at the same time. TO DO: Brake may require separate (or additional) controller
	throttle = max(0.0, pid_throttle_output)
	brake = BRAKE_MAX*min(0.0, pid_throttle_output)

	#Steer uses provided yaw controller
	#angular_velocity = self.lowpass_angle.filter(angular_velocity)
	steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_linear)
	#Calculate lateral cross track error for steering control
	#vector = [way_points[1][0] - way_points[0][0], way_points[1][1] - way_points[0][1]]
	#angle = atan(vector[1] / vector[0])
	#cte = -sin(angle)*(pose[0] - way_points[0][0]) + cos(angle)*(pose[1] - way_points[0][1])
	#pid_steer_output = self.pid_steer.step(-cte, time_elapsed)
	#steer = max( min( steer + pid_steer_output, self.max_angle), -self.max_angle)

	#Apply low pass filter to smooth out steering
	steer = self.lowpass_steer.filter(steer)
	# Return throttle, brake, steer
	return throttle, brake, steer
