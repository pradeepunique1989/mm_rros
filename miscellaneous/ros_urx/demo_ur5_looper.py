#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Twist, Vector3, PointStamped
import urx
import time
import math as m
import numpy as np
from datetime import datetime, timedelta
from gripperIO import GripperIO
from tf.transformations import euler_from_quaternion,euler_from_matrix
from ur5_inv_kin import ur5
import copy
def totimestamp(dt, epoch=datetime(1970,1,1)):
    td = dt - epoch
    return (td.microseconds + (td.seconds + td.days * 86400) * 10**6) / 10**6 

robotiq = GripperIO(15)
rob = urx.Robot("192.168.1.9")
rob.set_tcp((0, 0, 0, 0, 0, 0))
time.sleep(0.2)
robotiq.activate()
time.sleep(5)
robotiq.set_speed(50)
robotiq.go_to(1)
ur = ur5()



#########################################################################################
#########################################################################################
############################### joints ##################################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# List of all the joints                                                                   |
#===========================================================================================


J00 = [-86.58,-101.56,98.37,-105.28,-92.34,90.56]
J10 = [-86.58,-101.56,98.37,-105.28,-92.34,90.56]
J20 = [-4.15, -60.28, 33.84,-65.07, -87.67,90.24]

#pre drop
J30 = [-4.25,-65.01,64.66,-1.1,82.48,-4.84]

#drop
J40 = [-4.6,-52.84,70.43,-12.98,80.86,-3.51]

#dropback
J50 = [-5.91,-71.33,99.05,-23.09,79.44,-3.65]


J60 = [-86.58,-85.27,97.41,-91.63-10,82.34,-90.28]
J70 = [-86.58,-85.27,97.41,-91.63-25,82.34,-90.28]
J80 = [-86.58,-85.27,97.41,-91.63-5, 82.34,-90.28]



J0 = [x*m.pi/180.0 for x in J00]
J1 = [x*m.pi/180.0 for x in J10]
J2 = [x*m.pi/180.0 for x in J20]
J3 = [x*m.pi/180.0 for x in J30]
J4 = [x*m.pi/180.0 for x in J40]
J5 = [x*m.pi/180.0 for x in J50]
J6 = [x*m.pi/180.0 for x in J60]
J7 = [x*m.pi/180.0 for x in J70]
J8 = [x*m.pi/180.0 for x in J80]



j_target1 = J3
j_target2 = []
tolerance = 0.09
gripper_close_pose = 200
grasp_pose = [0,0,0,0,0,0]

t01 = 5
t12 = 10
t23 = 10
t34 = 6
t45 = 6
t56 = 6
t67 = 6
t78 = 6
t80 = 6

#########################################################################################

def table1_callback(data):
    
    global J0
    global J1
    global J2
    global J3
    global J4
    global J5
    global J6
    global J7
    global J8

    global t01 
    global t12 
    global t23 
    global t34 
    global t45 
    global t56 
    global t67 
    global t78 
    global t80

    global base_go_to_table2_pub

    global tolerance
    global gripper_close_pose
    global solution_found
    global j_works
    global grasp_pose

    rob.movej(J1, 0.5, 1.0, wait=False)
    rospy.sleep(t01)

    rob.movej(J2, 0.5, 1.0, wait=False)
    rospy.sleep(t12)
    print 'here'
    while solution_found == 0:
        a = 1
    print 'solfound'
    pre_grasp_pose = copy.copy(j_works)
    grasp_pose2 = copy.copy(grasp_pose)
    print 'grasp pose', grasp_pose2
    rob.movej(pre_grasp_pose, 0.5, 1.0, wait=False)

    pre_grasp_pose_reached = 0
    grasp_pose_reached = 0
    lift_pose_reached = 0

    joints = rob.getj()
    while pre_grasp_pose_reached == 0:
        joints = rob.getj()
        diff = [a_i - b_i for a_i, b_i in zip(joints, pre_grasp_pose)]
        print 'diff',diff
        if (abs(joints[0] - pre_grasp_pose[0]) < tolerance) and (abs(joints[1] - pre_grasp_pose[1]) < tolerance) and (abs(joints[2] - pre_grasp_pose[2]) < tolerance) and (abs(joints[3] - pre_grasp_pose[3]) < tolerance) and (abs(joints[4] - pre_grasp_pose[4]) < tolerance) and (abs(joints[5] - pre_grasp_pose[5]) < tolerance):
                pre_grasp_pose_reached = 1
                rospy.sleep(0.5)

    pre_grasp_pose_from_robot = rob.get_pose()
    grasp_pose2_correct_R = pre_grasp_pose_from_robot.copy()
    lift_pose = pre_grasp_pose_from_robot.copy()
    
    grasp_pose2_correct_R.pos = grasp_pose2

    lift_pose2 = copy.copy(grasp_pose2)
    lift_pose2[2] += 0.15
    lift_pose.pos = lift_pose2

    rob.movel(grasp_pose2_correct_R, 0.2, 0.5, wait=False)

    while grasp_pose_reached == 0:
        robposenow = rob.get_pose()
        diff = [a_i - b_i for a_i, b_i in zip(robposenow.pos, grasp_pose2)]
        print 'diff grasp pose -- \t',diff
        if (abs(robposenow.pos[0] - grasp_pose2[0]) < tolerance) and (abs(robposenow.pos[1] - grasp_pose2[1]) < tolerance) and (abs(robposenow.pos[2] - grasp_pose2[2]) < tolerance):
                grasp_pose_reached = 1
                rospy.sleep(0.5)
                robotiq.go_to(gripper_close_pose)
                rospy.sleep(2)

    rob.movel(lift_pose, 0.2, 0.5, wait=False)

    while lift_pose_reached == 0:
        robposenow = rob.get_pose()
        if (abs(robposenow.pos[0] - lift_pose2[0]) < tolerance) and (abs(robposenow.pos[1] - lift_pose2[1]) < tolerance) and (abs(robposenow.pos[2] - lift_pose2[2]) < tolerance):
                lift_pose_reached = 1
                rospy.sleep(1.0)


    rob.movej(J1, 0.5, 1.0, wait=False)
    final_pose_reached = 0
    joints = rob.getj()
    while final_pose_reached == 0:
        joints = rob.getj()
        diff = [a_i - b_i for a_i, b_i in zip(joints, J1)]
        print 'diff',diff
        if (abs(joints[0] - J1[0]) < tolerance) and (abs(joints[1] - J1[1]) < tolerance) and (abs(joints[2] - J1[2]) < tolerance) and (abs(joints[3] - J1[3]) < tolerance) and (abs(joints[4] - J1[4]) < tolerance) and (abs(joints[5] - J1[5]) < tolerance):
                final_pose_reached = 1
                rospy.sleep(1.0)

    pre_grasp_pose_reached = 0
    grasp_pose_reached = 0
    lift_pose_reached = 0
    final_pose_reached = 0
    solution_found = 0

    msg1 = Bool()
    msg1.data = True
    base_go_to_table2_pub.publish(msg1)



#########################################################################################



def table2_callback(data):
    
    global J0
    global J1
    global J2
    global J3
    global J4
    global J5
    global J6
    global J7
    global J8

    global t01 
    global t12 
    global t23 
    global t34 
    global t45 
    global t56 
    global t67 
    global t78 
    global t80

    global base_go_to_table1_pub 

    rob.movej(J3, 0.5, 1.0, wait=False)
    rospy.sleep(t56)

    rob.movej(J4, 0.5, 1.0, wait=False)
    rospy.sleep(t67)

    robotiq.go_to(0)
    rospy.sleep(2)

    rob.movej(J5, 0.5, 1.0, wait=False)
    rospy.sleep(5)

    rob.movej(J3, 0.5, 1.0, wait=False)
    rospy.sleep(8)

    rob.movej(J1, 0.5, 1.0, wait=False)
    final_pose_reached = 0
    joints = rob.getj()
    while final_pose_reached == 0:
        joints = rob.getj()
        diff = [a_i - b_i for a_i, b_i in zip(joints, J1)]
        print 'diff',diff
        if (abs(joints[0] - J1[0]) < tolerance) and (abs(joints[1] - J1[1]) < tolerance) and (abs(joints[2] - J1[2]) < tolerance) and (abs(joints[3] - J1[3]) < tolerance) and (abs(joints[4] - J1[4]) < tolerance) and (abs(joints[5] - J1[5]) < tolerance):
                final_pose_reached = 1
                rospy.sleep(0.5)

    msg2 = Bool()
    msg2.data = True
    base_go_to_table1_pub.publish(msg2)


#########################################################################################

part_pose = [[0,0,0],[0,0,0,0]]



def good_ik_sols(angles):
    theta = []

    for i in range(np.shape(angles)[1]):
        if angles[1, i] < 0 and angles[1, i] >= -m.pi / 2 and angles[2, i] > 0:
            theta.append(angles[:, i].reshape(1, np.shape(angles)[0]))
        elif angles[1, i] < m.pi / 2 and angles[1, i] >= -m.pi and angles[2, i] < 0:
            theta.append(angles[:, i].reshape(1, np.shape(angles)[0]))
    return theta


solution_found = 0

def aruco_callback(data):
    global part_pose
    global j_works
    global grasp_pose
    global solution_found

    part_pose[0][0] = data.pose.position.x
    part_pose[0][1] = data.pose.position.y
    part_pose[0][2] = data.pose.position.z

    part_pose[1][0] = data.pose.orientation.x
    part_pose[1][1] = data.pose.orientation.y
    part_pose[1][2] = data.pose.orientation.z
    part_pose[1][3] = data.pose.orientation.w

    cxp =  part_pose[0][1]
    cyp = -part_pose[0][0]
    czp =  part_pose[0][2]

    (r,p,y) = euler_from_quaternion(part_pose[1])

    t = y

    cwee = np.array( [ [ 1.0, 0.0, 0.0, -0.06 ], 
                       [ 0.0, 1.0, 0.0,  0.0  ],
                       [ 0.0, 0.0, 1.0,  0.015],
                       [ 0.0, 0.0, 0.0,  1.0  ] ] )


    pwc  = np.array( [ [ m.cos(t), -m.sin(t), 0.0, cxp ], 
                       [ m.sin(t), m.cos(t),  0.0, cyp ],
                       [ 0.0,      0.0,       1.0, czp ],
                       [ 0.0,      0.0,       0.0, 1.0 ], ] )

    pwee = np.matmul(cwee,pwc)

    # print 'pwee',pwee

    joints = rob.getj()

    eewb = np.array(ur.fwd_kin(joints))
    
    # print 'eewb',eewb

    pwb = np.matmul(eewb,pwee)

    # print 'pwb',pwb

    (r2,p2,yaw2) = euler_from_matrix(pwb,'sxyz')

    (r22,p22,yaw22) = euler_from_matrix(pwb,'szyx')


    # print 'yaw2',yaw2

    offset = 0.4
    px = pwb[0,3] + offset*m.cos(yaw2)
    py = pwb[1,3] + offset*m.sin(yaw2)
    pz = pwb[2,3]

    grasp_pose = [ pwb[0,3]+0.18*m.cos(yaw2), pwb[1,3]+0.18*m.sin(yaw2), pwb[2,3]+0.03]

    target_rot_mat  = np.array( [ [  m.sin(yaw2), 0.0, -m.cos(yaw2),  px ], 
                                  [ -m.cos(yaw2), 0.0, -m.sin(yaw2),  py ],
                                  [          0.0, 1.0,          0.0,  pz ],
                                  [          0.0, 0.0,          0.0, 1.0 ], ] )

    # print 'target_rot_mat',target_rot_mat

    jt = good_ik_sols(ur.inv_kin(target_rot_mat))

    jt_rad = np.multiply(180.0/m.pi,jt)

    # print 'sol j',jt_rad

    for i in range(len(jt_rad)):
        for j in range(len(jt_rad[i])):
            j_test = jt_rad[i][j]
            if (abs(j_test[5]) < 20.0) and (j_test[4] > 0.0) and (j_test[4] < 90.0) and (j_test[3] > -90.0) and (j_test[3] < 0.0) and (j_test[2] > 0.0) and (j_test[2] < 180.0) and (j_test[1] > -90.0) and (j_test[1] < 0.0) and (j_test[0] > -90.0) and (j_test[0] < 90.0):
                solution_found = 1
                j_works = jt[i][j]
                # print 'sol found cb',solution_found

#########################################################################################

def ur5_motion():
    global base_go_to_table1_pub
    global base_go_to_table2_pub

    rospy.init_node('demo_pick_and_place_loop_ur5', anonymous=True)
    rospy.Subscriber("base_reached1", Bool, table1_callback)
    rospy.Subscriber("base_reached2", Bool, table2_callback)
    rospy.Subscriber("aruco_single/pose", PoseStamped, aruco_callback)
    base_go_to_table1_pub = rospy.Publisher('base_go_to_table1', Bool, queue_size = 50)
    base_go_to_table2_pub = rospy.Publisher('base_go_to_table2', Bool, queue_size = 50)
    rospy.spin()

if __name__ == '__main__':
    ur5_motion()
