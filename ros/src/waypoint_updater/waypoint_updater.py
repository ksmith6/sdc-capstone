#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32

import math
import tf
from scipy.interpolate import CubicSpline

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # 200 # Number of waypoints we will publish. You can change this number
SLOW_DIST = 20. # (in meters) Distance from closest traffic light must be for car to start slowing down
STOP_DIST = 4. # (in meters) Distance from closest traffic light to decide whether to top or go through intersection

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')
		self.base_waypoints = None
		self.final_waypoints = None
		self.current_pose = None
		self.next_waypoint_index = None
		self.light_wp = None

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
		#rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
		self.base_waypoints = rospy.wait_for_message('/base_waypoints', Lane).waypoints #Only need to get base_waypoints once
		#rospy.Subscriber('/current_velocity', TwistStamped, callback=self.current_velocity_cb, queue_size=1)
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		self.loop()
		#rospy.spin()

	def pose_cb(self, msg):
		self.current_pose = msg.pose
	
	def waypoints_cb(self, waypoints):
		self.base_waypoints = waypoints.waypoints

	def current_velocity_cb(self, msg):
		self.current_velocity = msg.twist.linear.x

	def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message
		self.light_wp = msg.data

	def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass

	def loop(self):
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.base_waypoints and self.current_pose and self.light_wp: # and self.current_velocity:				
				start_idx = self.closest_waypoint(self.current_pose.position)
				# If this waypoint is behind the current pose then update to next waypoint
				self.next_waypoint_index = self.ahead_waypoint(start_idx)
				self.publish()
			rate.sleep()

	def publish(self):
		final_waypoints_msg = Lane()
		final_waypoints_msg.header.stamp = rospy.Time.now()
		self.set_final_waypoints()
		self.set_final_waypoints_speed()
		final_waypoints_msg.waypoints = self.final_waypoints
		self.final_waypoints_pub.publish(final_waypoints_msg)

	def set_final_waypoints(self):
		self.final_waypoints = self.base_waypoints[self.next_waypoint_index: self.next_waypoint_index + LOOKAHEAD_WPS]
		rem_points = LOOKAHEAD_WPS - len(self.final_waypoints)
		if rem_points > 0: self.final_waypoints = self.final_waypoints + self.base_waypoints[0:rem_points]
		
	def set_final_waypoints_speed(self):
		if self.light_wp < 0:
			for wp in self.final_waypoints: self.set_waypoint_velocity(wp, 4.47) #Accelelerate to top speed
			pass

		dist = self.distance(self.base_waypoints, self.next_waypoint_index, self.light_wp)
		rospy.logwarn("Next wp: %s, Next TL wp: %s, distance: %s",self.next_waypoint_index, self.light_wp, dist)
		if dist <= STOP_DIST: 
			for wp in self.final_waypoints: self.set_waypoint_velocity(wp, 0.0) #Decelerate to a stop
		else: 
			for wp in self.final_waypoints: self.set_waypoint_velocity(wp, 4.47) #Accelelerate to top speed
			
	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoint, velocity):
		waypoint.twist.twist.linear.x = velocity

	def distance(self, waypoints, wp1, wp2):
		dist, wp3 = 0.0, -1
		#In case of wraparound
		if wp2 < wp1: wp3, wp2 = wp2, len(waypoints)-1
		for i in xrange(wp1,wp2):
			dist += self.distance_between_points(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
		for i in xrange(-1,wp3):
			dist += self.distance_between_points(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
		return dist

	def distance_between_points(self, a, b):
		dist = math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
		return dist

	def closest_waypoint(self, position):
		return min(xrange(len(self.base_waypoints)), key = lambda p: self.distance_between_points(position, self.base_waypoints[p].pose.pose.position))

	def get_euler_yaw(self):
		quaternion = (
			self.current_pose.orientation.x,
			self.current_pose.orientation.y,
			self.current_pose.orientation.z,
			self.current_pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		return euler[2]

	def ahead_waypoint(self, wp_idx):
		ahead_idx = wp_idx
		map_wp_x = self.base_waypoints[wp_idx].pose.pose.position.x
		map_wp_y = self.base_waypoints[wp_idx].pose.pose.position.y

		x,y = self.current_pose.position.x, self.current_pose.position.y
		yaw = self.get_euler_yaw()

		localize_x = (map_wp_x - x) * math.cos(yaw) + (map_wp_y - y) * math.sin(yaw)
		if localize_x < 0.0: ahead_idx = ahead_idx + 1

		self.ahead_waypoint_index = ahead_idx
		return ahead_idx

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
