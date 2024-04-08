#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import datetime

def bool_callback(msg):
    #save to file
    if(msg.data == 1):
        f = open("water_log.txt", "a")
        f.write("Water strike detected at %s.\n" %(datetime.datetime.now().time()))
        print("Water strike detected at ", datetime.datetime.now().time())
        f.close()

def listener():
    rospy.init_node('water_logger')
    sub = rospy.Subscriber('water_log', Int8, bool_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass