#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, Twist

steering = 0
throttle = 0
emergency_stop = 0

def twist_callback(msg):
    global steering, throttle, emergency_stop
    steering = msg.angular
    throttle = msg.linear
    emergency_stop = msg.buttons[9]

if __name__ == '__main__':
    try:
        servo_values = Float32MultiArray(data=[-1.0, 1.0, 0.0, 0.0, 0.0, 0.0])

        #accessing joystick (in the future will be motion planner) and publishing to arduino
        pub = rospy.Publisher("/cmd_vel1", Float32MultiArray, queue_size=10)
        sub = rospy.Subscriber("/cmd_vel", Twist, twist_callback)

        rospy.init_node('ros__test')
        rospy.loginfo("ros_test is started")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            servo_values.data[0] = steering
            servo_values.data[1] = throttle
            servo_values.data[2] = emergency_stop
            rospy.loginfo(servo_values)
            pub.publish(servo_values)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass