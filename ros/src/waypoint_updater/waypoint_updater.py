#!/usr/bin/env python

import copy

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb )

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.carpose = None
        self.base_waypoints = None
        self.waypoints_tree = None
        self.waypoints_2d = []
        self.nearest_idx = -1
        self.next_red_light_idx = -1

        self.loop()
        


    def loop( self ):
        rate = rospy.Rate( 50 )

        while not rospy.is_shutdown():
            if self.carpose is not None and self.waypoints_tree is not None and len( self.waypoints_2d ) > 0 :
                self.nearest_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints( self.nearest_idx )
            rate.sleep()


    def get_closest_waypoint_idx( self ):
        if self.carpose is not None and self.waypoints_tree is not None and len( self.waypoints_2d ) > 0:
            nearest_idx = self.waypoints_tree.query( self.carpose, 1 )[1]
            
            closest_coord = np.array( self.waypoints_2d[ nearest_idx ] )
            prev_coord = np.array( self.waypoints_2d[ nearest_idx -1 ] )

            #Vector of a segment of the route
            route_vect = closest_coord - prev_coord 

            #Vector frmo the closest waypoint to the car poseition
            pos_vect = np.array( self.carpose )
            pos_vect = pos_vect - closest_coord

            #Evaluating if the car is "ahead" of "before" the nearest vector
            val = np.dot( route_vect, pos_vect )

            if val > 0 :
                nearest_idx = ( (nearest_idx + 1) % len( self.waypoints_2d ) )

            return nearest_idx

        return -1


    def pose_cb(self, msg):
        self.carpose = [msg.pose.position.x, msg.pose.position.y]
        
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

        if( self.waypoints_tree is None ):

            for waypoint in waypoints.waypoints:
                p2d = [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                self.waypoints_2d.append( p2d )


        self.waypoints_tree = KDTree( self.waypoints_2d )


    def publish_waypoints( self, nearest_idx ):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = copy.deepcopy( self.base_waypoints.waypoints[ nearest_idx:nearest_idx+LOOKAHEAD_WPS] )

        redlight_idx = self.next_red_light_idx - nearest_idx 
    
        if( self.next_red_light_idx != -1 and redlight_idx < 200 ):

            for i in range( len( lane.waypoints ) ):

                new_speed = lane.waypoints[i].twist.twist.linear.x

                if( i < redlight_idx ):
                    dist = self.distance( lane.waypoints, i, redlight_idx  )

                    if( dist < 60.0 ):

                        new_speed = 11.1 - (60 -  dist)/((60-30.0)/11.1)
                        if( new_speed < 0.0 ):
                            new_speed = 0.0
                        
                else: 
                    new_speed = 0.0
            
                lane.waypoints[i].twist.twist.linear.x = new_speed


        self.final_waypoints_pub.publish( lane )

    def traffic_cb(self, msg):
        self.next_red_light_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
