#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
SLOW_DIST = 6. # (in meters) Distance from closest traffic light must be for car to start slowing down
STOP_DIST = 2. # (in meters) Distance from closest traffic light to decide whether to top or go through intersection
RED = 0

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')
		self.base_waypoints = None
		self.final_waypoints = None
		self.current_pose = None
		self.next_waypoint_index = None
		self.tl_waypoints = None
		self.tl_waypoint_index = None
		self.tl_state = None

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
		sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_cb, queue_size=1)

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		self.loop()
		#rospy.spin()

	def pose_cb(self, msg):
		self.current_pose = msg.pose
	
	def waypoints_cb(self, waypoints):
		self.base_waypoints = waypoints.waypoints

	def traffic_light_cb(self, msg):
		# Temp Callback for /tl_waypoint message. To remove
		self.tl_waypoints = msg.lights

	def traffic_cb(self, msg):
        # TODO: Callback for /tl_waypoint message. Implement
		pass

	def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it
        # later
		pass

	def loop(self):
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.base_waypoints and self.current_pose and self.tl_waypoints:
				start_idx = self.closest_waypoint(self.current_pose.position)
				# If this waypoint behind the current pose then update to next waypoint
				self.next_waypoint_index = self.ahead_waypoint(self.current_pose.position, start_idx)
				self.closest_tl()
				self.publish()
			rate.sleep()

	def publish(self):
		final_waypoints_msg = Lane()
		final_waypoints_msg.header.stamp = rospy.Time.now()
#		rospy.logwarn("len(base_waypoints): %s, pose %s", len(self.base_waypoints), self.current_pose)
#		rospy.logwarn("nxt_wp_idx: %s, tl_wp_idx: %s", self.next_waypoint_index, self.tl_waypoint_index)
		self.set_final_waypoints()
		self.set_final_waypoints_speed()
		final_waypoints_msg.waypoints = self.final_waypoints
		self.final_waypoints_pub.publish(final_waypoints_msg)

	def set_final_waypoints(self):
		self.final_waypoints = self.base_waypoints[self.next_waypoint_index: self.next_waypoint_index + LOOKAHEAD_WPS]
		rem_points = LOOKAHEAD_WPS - len(self.final_waypoints)
		if rem_points > 0: self.final_waypoints = self.final_waypoints + self.base_waypoints[0:rem_points]
		
	def set_final_waypoints_speed(self):
		dist = self.distance(self.base_waypoints, self.next_waypoint_index, self.tl_waypoint_index)
		rospy.logwarn("Next wp: %s, Next TL wp: %s, distance: %s",self.next_waypoint_index, self.tl_waypoint_index, dist)
		if dist <= STOP_DIST and self.tl_state is RED: speed = 0.0
		elif dist <= SLOW_DIST and dist > STOP_DIST: speed = 2.2
		else: speed = 4.47 
		for wp in self.final_waypoints: wp.twist.twist.linear.x = speed

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

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
		closest_idx, min_dist = 0, 99999
		for idx, wp in enumerate(self.base_waypoints):
			dist = self.distance_between_points(position, wp.pose.pose.position)
			if dist < min_dist: closest_idx, min_dist = idx, dist
		return closest_idx

	def get_euler_yaw(self):
		quaternion = (
			self.current_pose.orientation.x,
			self.current_pose.orientation.y,
			self.current_pose.orientation.z,
			self.current_pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		return euler[2]

	def ahead_waypoint(self, position, wp_idx):
		ahead_idx = wp_idx
		map_wp_x = self.base_waypoints[wp_idx].pose.pose.position.x
		map_wp_y = self.base_waypoints[wp_idx].pose.pose.position.y

		x,y = self.current_pose.position.x, self.current_pose.position.y
		yaw = self.get_euler_yaw()

		localize_x = (map_wp_x - x) * math.cos(yaw) + (map_wp_y - y) * math.sin(yaw)
		if localize_x < 0.0: ahead_idx = ahead_idx + 1

		self.ahead_waypoint_index = ahead_idx
		return ahead_idx

	def closest_tl(self):
		closest_idx = min(xrange(len(self.tl_waypoints)), key = lambda p: self.distance_between_points(self.tl_waypoints[p].pose.pose.position, self.current_pose.position))
		next_idx = (closest_idx+1)%len(self.tl_waypoints)
		p1 = self.tl_waypoints[closest_idx].pose.pose.position
		p2 = self.tl_waypoints[next_idx].pose.pose.position
		angle = math.atan2(p2.y-p1.y,p2.x-p1.x)
		x,y = self.current_pose.position.x, self.current_pose.position.y
		localize_x = (x - p1.x) * math.cos(angle) + (y - p1.y) * math.sin(angle)
		if localize_x > 0.0: closest_idx = next_idx
		self.tl_waypoint_index = self.closest_waypoint(self.tl_waypoints[closest_idx].pose.pose.position) - 32 #estimated offset
		self.tl_state = self.tl_waypoints[closest_idx].state

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
