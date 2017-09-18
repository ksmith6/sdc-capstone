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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane,
                         self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        # below
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                         self.traffic_light_gt_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = None
        self.current_pose = None
        self.traffic_lights = None
        self.traffic_waypoint_index = None
        self.old_traffic_waypoint_index = None
        self.ahead_waypoint_index = None
        self.slow_down_traj = None
        rospy.spin()

    def pose_cb(self, msg):

        # rospy.logwarn("Pose CB: (%s, %s, %s)",
        #              msg.pose.position.x,
        #              msg.pose.position.y,
        #              msg.pose.position.z)

        # TODO: Implement
        if self.base_waypoints == None:
            return

        current_pose = msg.pose
        self.current_pose = current_pose
        current_base_waypoints = self.base_waypoints

        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()

        start_idx = self.closest_waypoint(current_pose.position)

        # Is this waypoint behind the current pose then update to next waypoint
        next_idx = self.ahead_waypoint(current_pose.position, start_idx)

        size_of_waypoints = len(self.base_waypoints)
        final_proposed_waypoints = []
        if ((next_idx + LOOKAHEAD_WPS) <= size_of_waypoints):
            final_proposed_waypoints = current_base_waypoints[
                next_idx: next_idx + LOOKAHEAD_WPS]
            # rospy.logwarn("Publish waypoints final from [%s, %s)",
            #              next_idx,
            #              next_idx + LOOKAHEAD_WPS)
        else:
            final_proposed_waypoints = current_base_waypoints[next_idx : size_of_waypoints] + \
                current_base_waypoints[
                    0: (next_idx + LOOKAHEAD_WPS) - size_of_waypoints]
            # rospy.logwarn("Publish waypoints final modulo with start idx [%s, %s) and [0, %s)",
            #              next_idx,
            #              size_of_waypoints,
            #              ((next_idx + LOOKAHEAD_WPS) - size_of_waypoints))

        final_waypoints_msg.waypoints = final_proposed_waypoints

        if (self.traffic_waypoint_index is not None) and (self.slow_down_traj is not None):
            if (next_idx >= self.traffic_waypoint_index + 1):
                rospy.logwarn("Resetting the slow down trajectory since next_wp (%s) passed the traffic light (%s)",
                              next_idx,
                              self.traffic_waypoint_index)
                self.slow_down_traj = None

        for i in range(len(final_waypoints_msg.waypoints)):
            final_waypoints_msg.waypoints[i].twist.twist.linear.x = 4.47

        if (self.slow_down_traj):
            traj_start_idx = self.slow_down_traj[0]
            offset = 0
            vel_vec = self.slow_down_traj[1]
            if ((traj_start_idx != self.ahead_waypoint_index) and (self.ahead_waypoint_index > traj_start_idx)):
                offset = self.ahead_waypoint_index - traj_start_idx
                curr_idx = 0

                rospy.logwarn("Set velocity starting from next_waypoint (%s) to vel idx offset (%s) len (%s)",
                              next_idx,
                              offset,
                              len(self.slow_down_traj[1]))

                while ((curr_idx + offset < len(self.slow_down_traj[1])) and
                       (curr_idx < LOOKAHEAD_WPS)):
                    idx = curr_idx + offset
                    final_waypoints_msg.waypoints[
                        curr_idx].twist.twist.linear.x = self.slow_down_traj[1][idx]
                    curr_idx = curr_idx + 1

            # Test - Remove afterwards - Send
            # rospy.logwarn("Updater Publish Waypoint CB: (%s, %s, %s)",
            #              final_waypoints_msg.waypoints[
            #                  0].pose.pose.position.x,
            #              final_waypoints_msg.waypoints[
            #                  0].pose.pose.position.y,
            #              final_waypoints_msg.waypoints[0].pose.pose.position.z)

        self.final_waypoints_pub.publish(final_waypoints_msg)

    def traffic_light_gt_cb(self, msg):

        if ((self.base_waypoints is None) or (self.current_pose is None) or (self.ahead_waypoint_index is None)):
            return

        self.traffic_lights = msg.lights

        closest_light_idx = self.closest_traffic_light(
            self.current_pose.position)
        tf_wp_idx = self.closest_waypoint(
            self.traffic_lights[closest_light_idx].pose.pose.position)

        if (self.traffic_waypoint_index == tf_wp_idx):
            return

        self.traffic_waypoint_index = tf_wp_idx
        # Test - Remove afterwards - Send
        # rospy.logwarn("Light CB: Current Position (%s, %s, %s) Closest Waypoint idx (%s) TF WP idx (%s) Closest Traffic Light is: (%s, %s, %s) state : %s",
        #             self.current_pose.position.x,
        #             self.current_pose.position.y,
        #             self.current_pose.position.z,
        #             self.ahead_waypoint_index, self.traffic_waypoint_index,
        #             self.traffic_lights[closest_light_idx].pose.pose.position.x,
        #             self.traffic_lights[closest_light_idx].pose.pose.position.y,
        #            self.traffic_lights[closest_light_idx].pose.pose.position.z,
        #             self.traffic_lights[closest_light_idx].state)

        if (self.traffic_waypoint_index >= self.ahead_waypoint_index):
            self.set_slow_down_trajectory()

    def set_slow_down_trajectory(self):

        next_wp_idx = self.ahead_waypoint_index
        if (self.slow_down_traj):
            old_start = self.slow_down_traj[0]
            velocities = self.slow_down_traj[1]

            rospy.logwarn("NONONO -SHOULD NOT BE HERE?: next wp idx (%s) TF-WP-idx (%s) old_start (%s) vel_len(%s) from-to (%s)",                 next_wp_idx,
                          self.traffic_waypoint_index,
                          old_start, len(velocities), next_wp_idx - old_start)

            self.slow_down_traj = [next_wp_idx,
                                   velocities[next_wp_idx - old_start:]]
        else:
            stop_distance = self.distance(
                self.base_waypoints, next_wp_idx, self.traffic_waypoint_index)
            # Maximum decelerations is 1.0
            stop_velocity = math.sqrt(2 * 1.0 * stop_distance)
            target_velocity = self.base_waypoints[
                next_wp_idx].twist.twist.linear.x
            v0 = min(stop_velocity, target_velocity)
            cubic_spline = CubicSpline(
                [0., stop_distance / 2, stop_distance], [v0, v0 / 2, 0.])
            tot_dist = 0
            velocities = []
            for i in range(next_wp_idx, self.traffic_waypoint_index + 1):
                vel_pt = cubic_spline(tot_dist).tolist()
                velocities.append(vel_pt)
                tot_dist += self.distance(self.base_waypoints, i, i + 1)
            self.slow_down_traj = [next_wp_idx, velocities]
            rospy.logwarn("First time build slow down Closest Waypoint from next wp idx (%s) to TF_WP_dx (%s) vel_len(%s)",
                          next_wp_idx,
                          self.traffic_waypoint_index,
                          len(velocities))

    def waypoints_cb(self, waypoints):
        # rospy.logwarn("Waypoint CB: (%s, %s, %s)",
        #              waypoints.waypoints[0].pose.pose.position.x,
        #              waypoints.waypoints[0].pose.pose.position.y,
        #              waypoints.waypoints[0].pose.pose.position.z)
        # rospy.logwarn("Number of Waypoints are (%s)",
        #              len(waypoints.waypoints))
        self.base_waypoints = waypoints.waypoints
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it
        # later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(
            (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_between_points(self, a, b):
        dist = math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        return dist

    def closest_waypoint(self, position):
        closest_idx = 0
        min_dist = 99999

        for idx, wp in enumerate(self.base_waypoints):
            dist = self.distance_between_points(
                position, wp.pose.pose.position)

            if dist < min_dist:
                closest_idx = idx
                min_dist = dist

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

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        yaw = self.get_euler_yaw()

        localize_x = ((map_wp_x - x) * math.cos(yaw) +
                      (map_wp_y - y) * math.sin(yaw))
        if (localize_x < 0.0):
            ahead_idx = ahead_idx + 1

        self.ahead_waypoint_index = ahead_idx
        return ahead_idx

    def closest_traffic_light(self, position):
        closest_idx = 0
        min_dist = 99999

        for idx, tl in enumerate(self.traffic_lights):
            dist = self.distance_between_points(
                position, tl.pose.pose.position)

            if dist < min_dist:
                closest_idx = idx
                min_dist = dist

        return closest_idx

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
