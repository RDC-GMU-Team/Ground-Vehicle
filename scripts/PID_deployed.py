#!/usr/bin/env python3

import rospy
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, Twist
import utils
from utilities import utils as uts
from sensor_msgs.msg import Imu as IMU
import math
import message_filters
import threading
import time
import numpy as np
from calling_model import ModelClass
from find_current_patch import FindCurrentPatch as image_processor

class HeadingCalculator:
    def __init__(self):
        #initialize the variables

        #self.waypoints = utils.read_csv("front_waypoint.csv")
        self.waypoints = utils.read_csv("cont_waypoints_last_saved.csv")
        self.navigation_started = False
        print(f"waypoints:{self.waypoints}")
        self.org_len = len(self.waypoints)
        
        self.pvt_sub = rospy.Subscriber('/f9p_rover/navpvt', NavPVT, self.pvt_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_stamped_pub = rospy.Publisher('/stamped_cmd_vel', TwistStamped, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', IMU, self.imu_callback, queue_size=100)
        # self.hunter_sub = rospy.Subscriber('/hunter_status', HunterStatus, self.hunter_callback, queue_size=1)
        self.uts = uts()

        self.twist = Twist()
        self.twist_stamped = TwistStamped()
        self.cache_pvt_msg = utils.CacheHeaderlessROSMessage(5)
        self.cache_imu_msg = utils.CacheHeaderlessROSMessage(5)
        
        self.rate = rospy.Rate(30)
        self.radius_at_equator = 6371 #km
        self.current_pos = []
        self.offset = 0
        self.previous_error = 0
        self.kp = 0.65
        self.kd = 0.41
        self.ki = 0.010
        self.integrate = 0
        self.goal_tolerance = 1.2
        self.ground_speed = 0.0
        self.steering_angle = 0.0
        #read from csv and store it in a self.waypoints 

    def pid(self):
        immediate_goal = self.waypoints[0]
        if HeadingCalculator.distance_between_two_points(self, self.current_pos, immediate_goal) > self.goal_tolerance:
            distance_between_two_points = HeadingCalculator.distance_between_two_points(self, self.current_pos, immediate_goal)
            if distance_between_two_points < 3.0:
                speed = 1.5
            else:
                speed = 4.8

            error = self.find_next_heading_to_waypoint(self.current_pos, self.current_heading, immediate_goal)

            heading = self.kp * error + self.kd * (error - self.previous_error) + self.ki * self.integrate
            self.previous_error = error
            self.integrate += error
            if self.integrate > 0.5:
                self.integrate = 0.5
            elif self.integrate < -0.5:
                self.integrate = -0.5 
            
            if abs(error) < 0.04:
                self.integrate = 0

            if heading > 0.5:
                heading = 0.5
            elif heading < -0.5:
                heading = -0.5
            self.previous_heading = heading
            # if abs(heading) > 0.5:
            self.twist.linear.x = speed 
            self.twist.angular.z = heading 
            self.twist_stamped.twist = self.twist
            self.twist_stamped.header.stamp = rospy.Time.now()
            
        else:
            #remove the waypoint from the lis
            print(f"Reached waypoint no : {self.org_len - len(self.waypoints) + 1}: {self.waypoints[0]}")
            self.waypoints.pop(0) 

    # def start_navigation(self):
    #     '''Take the first waypoint
    #        for each waypoint
    #           calculate the heading between current pos and waypoint
    #           if the heading is not in the range -45 to +45 degree then
    #                 just go to +45 or -45 for now
    #           else
    #                 start going to the heading
            
    #         check if the distance between current pos and waypoint is less than 0.5 meters
    #         then remove the waypoint from the list, and pick the first element from the 
    #     '''
    #     print("in the start navigation function")
    #     initilize_time = rospy.Time.now()
    #     while rospy.Time.now().to_sec() - initilize_time.to_sec() < 2.0:
    #         print("moving for 1 sec")
    #         self.twist.linear.x = 0.0
    #         #print(f"twist: {self.twist}")
    #         self.cmd_vel_pub.publish(self.twist)
    #         self.rate.sleep()

    #     # self.calculate_offset()

    #     print("done moving for 1 sec")
    #     while(len(self.waypoints) > 0):
    #         while(HeadingCalculator.distance_between_two_points(self, self.current_pos, immediate_goal) > 1.2):
    #                     print("Navigation is done")
    #     exit(0)

    # def publish_cmd_vel(self):
    #     '''This function publishes the cmd_vel to the rover,
    #        takes the twist as input, returns nothing'''
    #     while not rospy.is_shutdown() and len(self.waypoints) > 0:
    #         #print("publishing the cmd_vel")
    #         #print(f"the twist is{self.twist}")
    #         self.cmd_vel_pub.publish(self.twist)
    #         self.cmd_vel_stamped_pub.publish(self.twist_stamped)
    #         self.rate.sleep()
        

    #     print("No waypoints left, not publishing cmd_vel")

    #calculate distance between two gps waypoints
    # def check_acceptable_navigation(self, threshold_distance):
    #     '''This function checks if the distance between the current position and all the waypoints is less than the threshold distance,
    #        returns true/false'''
    #     for each in self.waypoints:
    #         if HeadingCalculator.distance_between_two_points(self, self.current_pos, each) > threshold_distance:
    #             return False

    #     return True 
    #     # return self.radius_at_equator * c * 1000 #in meters 
    
    def find_next_heading_to_waypoint(self, current_position, current_heading, waypoint):
        '''This function calculates the next heading to the waypoint,
        it takes the current_position, current_heading and waypoint as input,
        outputs the next heading in radians, if the heading is not in the range -45 to +45 degree then
        just go to +45 or -45 for now'''
        #first find the bearing using calculate heading function
        #then find the difference between current heading and bearing
        difference_in_heading = self.difference_heading(current_heading, math.degrees(HeadingCalculator.calculate_heading(current_position, waypoint)))

        #implement the logic to find the next heading
        if difference_in_heading >= -28 and difference_in_heading <= 28:
            if abs(difference_in_heading) < 1:
                return math.radians(0)
            return math.radians(difference_in_heading)
        elif difference_in_heading < -28:
            return math.radians(-28)
        else:
            return math.radians(28)

    #heading is in degrees
    def difference_heading(self, heading1, heading2):
        '''This function calculates the difference between two headings,
        rounds up the heading if the difference is greater than 180 degrees,
        needs input in degrees, returns the difference in degrees'''
        diff = heading1 - heading2
        if diff > 180:
            diff = diff - 360
        elif diff < -180:
            diff = diff + 360
        return diff

    #Calculate heading between two gps coordinates
    @staticmethod
    def calculate_heading(CoordinateA,CoordinateB):
        '''Calculates the heading between two gps coordinates,
           returns the heading in radians'''
        X = math.cos(CoordinateB[0]) * math.sin(CoordinateB[1] - CoordinateA[1])
        Y = math.cos(CoordinateA[0]) * math.sin(CoordinateB[0]) - math.sin(CoordinateA[0]) * math.cos(CoordinateB[0]) * math.cos(CoordinateB[1] - CoordinateA[1])

        heading = math.atan2(X,Y)
        return heading
    
    @staticmethod
    def distance_between_two_points(self, waypoint1, waypoint2):
        '''Calculate the distance between two gps coordinates
           using haversine formula, returns the distance in meters'''        
        lat1, lon1 = math.radians(waypoint1[0]), math.radians(waypoint1[1])
        lat2, lon2 = math.radians(waypoint2[0]), math.radians(waypoint2[1])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.asin(math.sqrt(a))
        return self.radius_at_equator * c * 1000 #in meters

    #change the values of current_pos and current_heading everytime the pvt_callback is called
    def pvt_callback(self, data):
        '''This function is called everytime the navigation message is received,
           it updates the current position and current heading of the rover, returns nothing'''
        self.current_pos = [data.lat * 1e-7, data.lon * 1e-7]
        self.ground_speed = data.gSpeed * 1e-3
        self.cache_pvt_msg.add_element(data.heading * 1e-5) 
        
    def imu_callback(self, data):
        temp_yaw = np.degrees(self.uts.quaternion_to_yaw(data.orientation)) + self.offset 
        temp_yaw = -1 * temp_yaw
        temp_yaw = utils.round_up_angles(temp_yaw) 
        self.current_heading = temp_yaw
        self.cache_imu_msg.add_element(self.current_heading)

    def hunter_callback(self, data):
        self.steering_angle = data.steering_angle

    # def calculate_offset(self):
    #     yaw_msgs = self.cache_imu_msg.get_all()
    #     heading_msgs = self.cache_pvt_msg.get_all()

    #     #     print(f"offset has been calculated at: {self.offset}")
    #     local_offset = 0
    #     for i in range(0, len(yaw_msgs)):
    #         local_offset = local_offset + utils.substract_two_angles(yaw_msgs[i], heading_msgs[i])

    #     # self.offset = local_offset / 10
    #     self.offset = 0 
    #     print(f"offset has been calculated at: {self.offset}")

if __name__ == '__main__':
    rospy.init_node('HeadingCalculator')
    #initialize the class
    HeadingCalculatorNode = HeadingCalculator()
    rospy_rate = rospy.Rate(50)
    while not rospy.is_shutdown:
        if len(HeadingCalculatorNode.waypoints) > 0:
        # if not HeadingCalculatorNode.navigation_started and len(HeadingCalculatorNode.current_pos) > 0:
            HeadingCalculatorNode.pid()
        else:
            HeadingCalculator.twist.linear.x = 0.0
            HeadingCalculator.twist.angular.z = 0.0
            print("No waypoints left, not publishing cmd_vel")
    
            # twist_publisher_thread.join()
        HeadingCalculator.cmd_vel_pub.publish(HeadingCalculator.twist)
        HeadingCalculator.cmd_vel_stamped_pub.publish(HeadingCalculator.twist_stamped)
        rospy_rate.sleep()
    rospy.spin()


