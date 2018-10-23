#!/usr/bin/env python  

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Twist, Vector3, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import *
import math as m
import time



#########################################################################################
#########################################################################################
############################### pose updater ############################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# updates pose when new pose is received                                                    |
#===========================================================================================

pose = [0,0]
initial_pose = [0,0]
first = 0

def pose_update(data):

    global pose
    global initial_pose
    global first

    if first == 1:
        initial_pose = pose
        first = 0

    if (pose[0] == 0) and (pose[1] == 0):
        first = 1

    pose[0] = data.pose.pose.position.y
    pose[1] = data.pose.pose.position.x
    debug_printer(debug,'pose',pose)


#########################################################################################



#########################################################################################
#########################################################################################
###################################### Debug mode #######################################
#########################################################################################
#########################################################################################

debug = 1

def debug_printer(debug,variable,variable_value):

    if debug == 1:

        print '======',variable,':',variable_value,'======'

    return 

debug_printer(debug,'DEBUG MODE', 'ACTIVATED')

#########################################################################################



#########################################################################################
#########################################################################################
############################### velocity setter #########################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# depending on distance, sets acceleration value                                           |
#===========================================================================================

min_vel = 0.04
max_vel = 0.25
max_acceleration = 0.003

def set_vel(distance,initial_distance,last_velocity):

    global min_vel
    global max_vel
    global max_acceleration

    dist_pct = ( ( initial_distance - distance ) / initial_distance ) * 100

    if dist_pct < 35.0:
    
        acceleration = max_acceleration
        velocity = min(last_velocity + acceleration,max_vel)
    
    elif dist_pct > 65.0:
    
        acceleration = -max_acceleration
        velocity = max(last_velocity + acceleration,min_vel)

    else:

        velocity = last_velocity

    return velocity


#########################################################################################



#########################################################################################
#########################################################################################
############################### reach ###################################################
#########################################################################################
#########################################################################################

distance_tolerance = 0.04 # in meters

#=========================================================================================== 
# for a given way point, go straight till arrival                                          |
#===========================================================================================

def reach_straight(waypoint):

    global pose
    global vel_pub

    global initial_pose

    global distance_tolerance
    global velocity
    global acc

    global debug
    global rate

    initial_distance = m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1])

    if pose[1] < waypoint[1]:
    	sign = 1
    else:
    	sign = -1

    msg = Twist()
    msg.linear.x = min_vel
    
    last_velocity = msg.linear.x

    while (m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1]) > distance_tolerance) and (not rospy.is_shutdown()):

        debug_printer(debug,'delta-distance',m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1]))
        distance = m.hypot(waypoint[0] - pose[0],waypoint[1] - pose[1])
        velocity = set_vel(distance,initial_distance,last_velocity)
        msg.linear.x = sign*velocity
        vel_pub.publish(msg)
        rate.sleep()
        last_velocity = velocity

    msg.linear.x = 0
    msg.angular.z = 0
    vel_pub.publish(msg)
    rospy.sleep(1)


#########################################################################################



if __name__ == '__main__':

    rospy.init_node('demo_pick_and_place_straight')
    rate = rospy.Rate(25) # 25hz
    rospy.Subscriber('odom_test',Odometry,pose_update)
    rospy.sleep(0.2)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 50)
    arm_pub = rospy.Publisher('ur5_run', Bool, queue_size = 50)
    table1 = [0.0,2.2]
    table2 = [0.0,0.0]
# loop 1
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 2
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 3
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 4
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 5
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 6
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 7
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 8
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 9
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

# loop 10
    reach_straight(table1)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

    reach_straight(table2)
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 0
    vel_pub.publish(msg)

    rospy.sleep(2.0)

