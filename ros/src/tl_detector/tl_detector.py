#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.waypoints_tree = None
        self.waypoints_tree = None
        self.waypoints_2d = []

        self.next_traffic_light_idx = -1
        self.next_traffic_light_state = TrafficLight.UNKNOWN

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.REFERENCE_ACCEL = 10/4.0 #(1300 is maximum deceleration. So let's aassume to decelerate 1/4 of this value)
        self.CROSSING_WIDTH = 32.0  #Distance from 

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

        if( self.waypoints_tree is None ):

            for waypoint in waypoints.waypoints:
                p2d = [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                self.waypoints_2d.append( p2d )


        self.waypoints_tree = KDTree( self.waypoints_2d )

    def traffic_cb(self, msg):
        self.lights = msg.lights

        car_position = -1
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
    

        light_index = -1
        next_light_state = TrafficLight.UNKNOWN
        for light in self.lights:
            light_index = self.get_closest_waypoint( light.pose.pose )

            if( light_index > car_position ):
                self.next_traffic_light_idx = light_index
                self.next_traffic_light_state = light.state
                break;
        

        distace = -1.0
        if( light_index != -1 ):
            distance = self.distance( self.base_waypoints.waypoints, car_position, self.next_traffic_light_idx )

        if( self.next_traffic_light_state == TrafficLight.UNKNOWN ):
            #Unknown, we should stop for caution
            self.upcoming_red_light_pub.publish( Int32( light_index ) )
        elif( distance > self.CROSSING_WIDTH*0.7 and self.next_traffic_light_state == TrafficLight.RED ):
            #Red, we shoud stop. If distance < 10, probably, we started to cross on yellow and it turned, keep going
            self.upcoming_red_light_pub.publish( Int32( light_index ) )
        elif(  distance > 50.0 and self.next_traffic_light_state == TrafficLight.YELLOW):
            #We are at least 30 meters and turned yellow, stop it
            self.upcoming_red_light_pub.publish( Int32( light_index ) )
        else:
            self.upcoming_red_light_pub.publish( Int32( -1 ) )


    ## Torricelli Formulas. It will calculate how many meters before a full stop
    # we need to start to decelerate given an constant accel,ref_accel
    def distance_for_stop( self, start_speed, ref_accel ):
        return  ( start_speed**2.0 )/( 2.0 * ref_accel )

    def image_cb(self, msg):
        #rospy.logwarn('hello')
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        carpose = [pose.position.x, pose.position.y]

        if self.waypoints_tree is not None and len( self.waypoints_2d ) > 0:
            nearest_idx = self.waypoints_tree.query( carpose, 1 )[1]
            
            closest_coord = np.array( self.waypoints_2d[ nearest_idx ] )
            prev_coord = np.array( self.waypoints_2d[ nearest_idx -1 ] )

            #Vector of a segment of the route
            route_vect = closest_coord - prev_coord 

            #Vector frmo the closest waypoint to the car poseition
            pos_vect = np.array( carpose )
            pos_vect = pos_vect - closest_coord

            #Evaluating if the car is "ahead" of "before" the nearest vector
            val = np.dot( route_vect, pos_vect )

            if val > 0 :
                nearest_idx = ( (nearest_idx + 1) % len( self.waypoints_2d ) )

            return nearest_idx
        return -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)


        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
