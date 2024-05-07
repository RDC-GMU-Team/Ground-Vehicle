import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from nav_msgs.msg import Path , Odometry
#from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import Imu, MagneticField
from ublox_msgs.msg import NavPVT
import time

import numpy as np
import math

from utils import utilities

from sensor_fusion import imu_processor

Wheel_Base = 0.32
Max_Speed = 1.0
Min_Speed = -1.0
Max_Steering_Angle = 1
Min_Streering_Angle = -1
goal_threshold = 0.5

class PathPlanner:
    def __init__(self):
        self.K = 1001
        self.T = 2
        self.dt = 0.25
        self.max_iter = 5
        i_angle = -218 #245 #, 345 , 170
        
        self.ut = utilities()
        self.goal = np.zeros(3, dtype=np.float32)
        self.trajectory = []
        self.paths = np.zeros((self.T, self.K, 3), dtype=np.float32)
        self.robot_pose = np.zeros(3, dtype=np.float32)
        self.robot_prv_pose = np.zeros(3, dtype=np.float32)
        self.prev_time = time.time()
        self.curr_pose = np.zeros((self.K, 3), dtype=np.float32)
        self.goal_init = False
        self.min_cost = 10000000.0
        self.gps_heading = 0
        self.heading = 0
        self.alpha = 0.00
        self.h_acc_th = 20
        self.gps_init = False
        self.i_x = 0
        self.i_y = 0
        self.nav_init = False
        
        #pid variables
        self.Kp = .05
        self.Ki = 0
        self.Kd = .005
        self.prev_error = 0
        self.integral = 0
        self.target_speed = .17
        self.ground_speed = 0

        #turning loop vars
        self.turning_lock = False 

        self.twist = Twist()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=100)
        rospy.Subscriber('/magnetometer', MagneticField, self.mag_callback, queue_size=1)
        rospy.Subscriber('/f9p_rover/navpvt', NavPVT, self.pvt_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)

        self.imu = imu_processor()
        yaw = self.ut.gps_to_normal_heading(i_angle - 90) 
        yaw = -self.ut.clamp_angle(yaw)

        q = self.ut.yaw_to_quaternion(yaw) 
        print(f"Initial Heading {np.degrees(yaw)}")
        
        self.imu.q = [q.w, q.x, q.y, q.z]
        self.imu.heading = np.degrees(yaw)
        self.gps_heading = yaw
        self.final_path = []
        self.next_waypoint = []
        self.goal_list = [[38.8277156, -77.3053649], [38.8277847, -77.3053991]]
        # self.robot_pose[2] = yaw 

        self.goal_list = self.ut.csv_to_goals('waypoints.csv')
        #for i in range(self.goal_list.shape[0]):
        #    self.goal_list[i] = self.ut.gps_to_xy(self.goal_list[i][0], self.goal_list[i][1])
        print("Path Planner Initialized waiting for goal")
        
    def compute(self):
        error = self.target_speed - self.ground_speed
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        if output < -0.01:
            output = -0.01
        if output > 0.05:
            output = 0.05
        return output

    def odom_callback(self, odom_msg):
        yaw = self.ut.quaternion_to_yaw(odom_msg.pose.pose.orientation)
        self.robot_pose = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw])
    
    def goal_callback(self, goal_msg):
        self.goal = np.array([goal_msg.pose.position.x, goal_msg.pose.position.y, self.ut.quaternion_to_yaw(goal_msg.pose.orientation)])
        self.goal_init = True
        print("Goal Received")

    def plan_path(self, start, goal):
        
        if goal is None:
            return
        
        self.curr_pose[:, :] = start
        
        
        if self.ut.get_dist(start, goal) < goal_threshold:
            return
        
        velocity = np.zeros(self.K, dtype=np.float32)
        velocity[:] = 0.5
        steering = np.linspace(Min_Streering_Angle, Max_Steering_Angle, self.K)
    
        trajectory = []
        self.min_cost = 10000000.0
        cost = 0.0

        #plan path till max iterations
        for iteration in range(self.max_iter):
            step = 0
            for t in range(self.T):
                change = self.ut.ackermann_model(velocity*2, steering*0.78, Wheel_Base, self.dt)
                pose = self.ut.to_world_se2(self.curr_pose, change)
                self.paths[t] = pose
                self.curr_pose = pose
                step += 1

                cost = self.ut.get_dist(pose, goal)
                mask = cost < goal_threshold
                velocity[mask] = 0.0
            
            min_index = np.argmin(cost)
            min_cost_dist = cost[min_index]
            for k in range(step):
                t_pose  = Pose()
                t_pose.position.x, t_pose.position.y, p_yaw = self.paths[k, min_index]
                t_pose.position.z = 0.0
                t_pose.orientation = self.ut.yaw_to_quaternion(p_yaw)

                if k > 0:
                    if self.ut.get_dist(self.paths[k-1, min_index], self.paths[k, min_index]) < 0.001:
                        continue
                    
                trajectory.append(t_pose)
            
            # if self.min_cost < min_cost_dist :
            #     break
            
            self.curr_pose[:] = self.paths[step-1, min_index]
            self.min_cost = min_cost_dist

        #publish the path
        # path_msg = Path()
        # path_msg.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID
        # path_msg.poses = [self.ut.create_pose_stamped(t_pose) for t_pose in trajectory]
        # self.path_pub.publish(path_msg)
        self.final_path = trajectory
        self.pose_to_goal
        self.nav_init = True
        #print(f"Path Published no. of points: {len(trajectory)}")   

    def imu_callback(self, imu_data):
        self.imu.imu_update(imu_data)
        self.imu.heading = (1-self.alpha) * np.radians(self.imu.heading)  + self.alpha *  self.gps_heading
        self.imu.heading = self.ut.clamp_angle(self.imu.heading)
        q = self.ut.yaw_to_quaternion(self.imu.heading)
        self.imu.q = [q.w, q.x, q.y, q.z] 
        self.robot_pose[2] = self.imu.heading
        self.imu.heading = np.degrees(self.imu.heading)
    
    def mag_callback(self, mag_data):
        self.imu.mag_update(mag_data)
        #pass
    
    def pvt_callback(self, data):
        self.robot_pose[:2] = self.ut.gps_to_xy(data.lat * 1e-7, data.lon * 1e-7)
        self.gps_heading = self.ut.gps_to_normal_heading(data.heading * 1e-5 - 90)


        if self.i_x == 0:
            self.i_x = self.robot_pose[0]
            self.i_y = self.robot_pose[1]
            self.goal_init = True
            self.pose_to_goal()
            

        self.robot_pose[0] -= self.i_x
        self.robot_pose[1] -= self.i_y
            
        t_diff = time.time() - self.prev_time 
        if  t_diff >= 1.5: 
            dx = self.robot_prv_pose[0] - self.robot_pose[0]
            dy = self.robot_prv_pose[1] - self.robot_pose[1]
            self.ground_speed = np.sqrt(dx**2 + dy**2)
            self.robot_prv_pose[:2] = self.robot_pose[:2]
            self.prev_time = time.time()
        
        self.ground_speed = self.ground_speed * 2.237
        r_pose = Pose()
        r_pose.position.x, r_pose.position.y = self.robot_pose[0], self.robot_pose[1]
        r_pose.orientation = self.ut.yaw_to_quaternion(self.robot_pose[2])
        r_pose = self.ut.create_pose_stamped(r_pose)
        self.pose_pub.publish(r_pose)

        h_acc = data.headAcc * 1e-5
        
        if h_acc <= self.h_acc_th:# and abs(np.degrees(self.imu.heading) - np.degrees(self.gps_heading)) <  self.h_acc_th*1.5:
            self.alpha = 0.0025
        else:
            self.alpha = 0.00025
        self.gps_init = True

        # print(f"True Heading {self.imu.heading}, GPS heading {np.degrees(self.gps_heading)}, GPS_acc {h_acc}")

    def get_next_goal(self):
        if len(self.goal_list) != 0:
            lat_lon = self.goal_list.pop()
            self.goal[:2] = self.ut.gps_to_xy(lat_lon[0], lat_lon[1])
            self.goal[0] -= self.i_x
            self.goal[1] -= self.i_y

    def pose_to_goal(self):
        if len(self.final_path) != 0:
            p = self.final_path.pop()
            x = p.position.x
            y = p.position.y
            phi = 0
            self.next_waypoint = [x, y , phi]
        else:
            self.get_next_goal()
            self.next_waypoint = self.goal


    def execute_path(self):
        # goal = self.trajectory.pop()
        # x, y, yaw = goal.position.x, goal.position.y, self.ut.quaternion_to_yaw(goal.orientation)
        # lon = -77.3052674
        # lat = 38.8277552
        # goal = np.zeros(3, dtype=np.float32)
        # goal[:2] = self.ut.gps_to_xy(lat, lon)
        # if self.nav_init == False:
        #     return
        if len(self.next_waypoint) == 0:
            return
        
        dist = self.ut.get_dist(self.robot_pose, self.next_waypoint)

        if dist < 1.5:
            if self.ut.get_dist(self.robot_pose, self.goal) < 1.5:
                self.get_next_goal()
            else:
                self.pose_to_goal()

            return
        
        x, y = self.next_waypoint[0], self.next_waypoint[1]
        del_x = x - self.robot_pose[0]
        del_y = y - self.robot_pose[1]
        target_angle = np.degrees(math.atan2(del_y, del_x))
        st_ang = target_angle - np.degrees(self.robot_pose[2])
        # st_ang = -st_ang
        if st_ang < -180:
            st_ang = (360 + st_ang)
        
        if st_ang > 180: 
            st_ang = -(360 - st_ang)
        
        st_ang = max(-45, min(45, st_ang))
        print(f"steering angle {st_ang:.2f}, current_heading {np.degrees(self.robot_pose[2])}, target_angle {target_angle:.2f}, distance {dist}, Gspeed {self.ground_speed} ")
        st_ang = self.ut.map_value(st_ang, -45,45,-1,1)

        self.twist.linear.x = 0.18 + self.compute() + 0.05 * abs(st_ang)
        self.twist.angular.z = -st_ang

        self.cmd_pub.publish(self.twist)

    def turn_loop(self):


if __name__ == '__main__':
    rospy.init_node("Nav_Planner", anonymous=True) # Initialize the node
    pl = PathPlanner()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if pl.goal_init:
            t = time.time()
            pl.plan_path(pl.robot_pose, pl.goal)
            print(time.time()-t)
            pl.execute_path()
        rate.sleep()



