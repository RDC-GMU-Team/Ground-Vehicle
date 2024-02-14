#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

steering = 0
throttle = 0
emergency_stop = 0

def joy_callback(msg):
    global steering, throttle, emergency_stop
    steering = msg.axes[0]
    throttle = msg.axes[4]
    emergency_stop = msg.buttons[9]
    # Speed information to change with buttons.
    # speed1 = msg.buttons[0]
    # speed2 = msg.buttons[1]
    # speed3 = msg.buttons[2]

if __name__ == '__main__':
    try:
        servo_values = Float32MultiArray(data=[-1.0, 1.0, 0.0, 0.0, 0.0, 0.0])

        #accessing joystick (in the future will be motion planner) and publishing to arduino
        pub = rospy.Publisher("/cmd_vel1", Float32MultiArray, queue_size=10)
        sub = rospy.Subscriber("/joy", Joy, joy_callback)

        rospy.init_node('ros__test')
        rospy.loginfo("ros_test is started")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # Speed information to change with buttons.
            # if (speed1 > 0):
            #     throttle = val1
            
            # elif (speed2 > 0):
            #     throttle = val2
            
            # elif (speed3 > 0):
            #     throttle = val3
            
                
            servo_values.data[0] = -steering
            servo_values.data[1] = throttle
            servo_values.data[2] = emergency_stop
            rospy.loginfo(servo_values)
            pub.publish(servo_values)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
