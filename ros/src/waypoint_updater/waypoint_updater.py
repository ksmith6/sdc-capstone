#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        # below

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = None
        rospy.spin()

    def pose_cb(self, msg):

        rospy.logwarn("Pose CB: (%s, %s, %s)",
                      msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z)

        # TODO: Implement
        if self.base_waypoints == None:
            return

        current_pose = msg.pose
        current_base_waypoints = self.base_waypoints

        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()

        start_idx = self.closest_waypoint(current_pose.position)
        size_of_waypoints = len(self.base_waypoints)
        final_proposed_waypoints = []
        if ((start_idx + LOOKAHEAD_WPS) <= size_of_waypoints):
            final_proposed_waypoints = current_base_waypoints[
                start_idx: start_idx + LOOKAHEAD_WPS]
            rospy.logwarn("Publish waypoints final from [%s, %s)",
                          start_idx,
                          start_idx + LOOKAHEAD_WPS)
        else:
            final_proposed_waypoints = current_base__waypoints[start_idx : size_of_waypoints] + \
                current_base__waypoints[
                    0: (start_idx + LOOKAHEAD_WPS) - size_of_waypoints]
            rospy.logwarn("Publish waypoints final modulo with start idx [%s, %s) and [0, %s)",
                          start_idx,
                          size_of_waypoints,
                          ((start_idx + LOOKAHEAD_WPS) - size_of_waypoints))

        final_waypoints_msg.waypoints = final_proposed_waypoints
        for i in range(len(final_waypoints_msg.waypoints)):
            final_waypoints_msg.waypoints[i].twist.twist.linear.x = 10

            # Test - Remove afterwards - Send
            rospy.logwarn("Updater Publish Waypoint CB: (%s, %s, %s)",
                          final_waypoints_msg.waypoints[
                              0].pose.pose.position.x,
                          final_waypoints_msg.waypoints[
                              0].pose.pose.position.y,
                          final_waypoints_msg.waypoints[0].pose.pose.position.z)

        self.final_waypoints_pub.publish(final_waypoints_msg)

    def waypoints_cb(self, waypoints):
        rospy.logwarn("Waypoint CB: (%s, %s, %s)",
                      waypoints.waypoints[0].pose.pose.position.x,
                      waypoints.waypoints[0].pose.pose.position.y,
                      waypoints.waypoints[0].pose.pose.position.z)
        rospy.logwarn("Number of Waypoints are (%s)",
                      len(waypoints.waypoints))
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
