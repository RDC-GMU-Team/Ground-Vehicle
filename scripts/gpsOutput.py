#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from ublox_msgs.msg import NavPVT
import utils

cache_pvt_msg = utils.CacheHeaderlessROSMessage(5)
current_pos = 0
ground_speed = 0

def pvt_callback(data):
        '''This function is called everytime the navigation message is received,
           it updates the current position and current heading of the rover, returns nothing'''
        global current_pos, ground_speed, cache_pvt_msg
        current_pos = [data.lat * 1e-7, data.lon * 1e-7]
        ground_speed = data.gSpeed * 1e-3
        cache_pvt_msg.add_element(data.heading * 1e-5)

if __name__ == '__main__':
    try:
        servo_values = Float32MultiArray(data=[-1.0, 1.0, 0.0, 0.0, 0.0, 0.0])

        #accessing joystick (in the future will be motion planner) and publishing to arduino
        sub = rospy.Subscriber('/f9p_rover/navpvt', NavPVT, pvt_callback)

        rospy.init_node('ros__test')
        rospy.loginfo("ros_test is started")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            print('Current pos: ' + current_pos + 'Ground speed: ' + ground_speed)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass