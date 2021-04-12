#!/usr/bin/env python

import copy

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
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
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb )
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb )

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.carpose = None
        self.twist = None
        self.base_waypoints = None
        self.waypoints_tree = None
        self.waypoints_2d = []
        self.nearest_idx = -1
        self.next_red_light_idx = -1

        self.REFERENCE_ACCEL = 10/4.0 #(1300 is maximum deceleration. So let's aassume to decelerate 1/4 of this value)
        self.CROSSING_WIDTH = 30.0  #Distance from 

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

    def twist_cb( self, msg ):
        self.twist = msg

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

            nominal_speed = lane.waypoints[0].twist.twist.linear.x

            #Distance needed for stop the car, given the current speed and an desired deceleration
            dist_for_stopping = self.distance_for_stop( nominal_speed, self.REFERENCE_ACCEL )

            if( self.twist is not None and self.twist.twist.linear.x > nominal_speed ):
                dist_for_stopping = self.distance_for_stop( self.twist.twist.linear.x, self.REFERENCE_ACCEL )            

            #Distancce in wich we need to have sttoped for the the next traffic light,
            dist_for_next_traffic_light = self.distance( lane.waypoints, 0, redlight_idx  ) + self.CROSSING_WIDTH

            accel = self.REFERENCE_ACCEL

            if( dist_for_next_traffic_light > 0 and  dist_for_next_traffic_light > dist_for_stopping ):
                #We do not have enought distance for stop at the desired reference accel, let's see
                #which accel we do need and use it for deceleration
                accel = self.get_accel( self.twist.twist.linear.x, dist_for_next_traffic_light )
                if( accel < self.REFERENCE_ACCEL ):
                    accel = self.REFERENCE_ACCEL



            for i in range( len( lane.waypoints ) ):

                new_speed = lane.waypoints[i].twist.twist.linear.x

                if( i < redlight_idx ):
                    #Distance between an waypoint and the point where we need to have stopped
                    dist = self.distance( lane.waypoints, i, redlight_idx  ) - self.CROSSING_WIDTH

                    if( dist < ( dist_for_stopping + self.CROSSING_WIDTH ) ):

                        new_speed = self.get_ref_speed( dist , accel )

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


    ## Torricelli Formulas. It will calculate how many meters before a full stop
    # we need to start to decelerate given an constant accel,ref_accel
    def distance_for_stop( self, start_speed, ref_accel ):
        return  ( start_speed**2.0 )/( 2.0 * ref_accel )

    ## Inverse of the Torricelli Formula
    # IT will let us plan wich speed to achieve given a distance, assuming that 
    # after the car has travelled the giben distance with the constant accel, it will
    # be stopped
    def get_ref_speed( self, distance, ref_accel ):
        a = distance * 2.0 * ref_accel
        if a >= 0 :
            return math.sqrt( a )
        return 0.0

    def get_accel( self, start_speed, distance ):
        return ( start_speed ** 2.0) / ( 2* distance )


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
