import sys
import copy
import rospy
import tf
import math
import numpy as np
from ur5_inv_kin import ur5
from time import time


def good_ik_sols(angles):
    theta = []
    for i in range(np.shape(angles)[1]):
        if angles[1, i] < 0 and angles[1, i] >= -math.pi / 2 and angles[2, i] > 0:
            theta.append(angles[:, i].reshape(1, np.shape(angles)[0]))
        elif angles[1, i] < math.pi / 2 and angles[1, i] >= -math.pi and angles[2, i] < 0:
            theta.append(angles[:, i].reshape(1, np.shape(angles)[0]))
    return theta


def solve_IK(pose):
    #(px,py,pz,ax,ay,az):
    # kin = Kinematics('ur5')
    ur = ur5()
    ax = math.radians(pose[3])
    ay = math.radians(pose[4])
    az = math.radians(pose[5])
    M = tf.transformations.euler_matrix(ax, ay, az, axes='sxyz')
    M[0, 3] = pose[0]
    M[1, 3] = pose[1]
    M[2, 3] = pose[2]
    # sols = ur.(np.array(M))
    # sols = kin.inverse(np.array(M))
    sols = ur.inv_kin(np.array(M))
    print sols
    # thetas = []
    # if len(sols) :
    #     print "IK solution not found"
    # else:
    thetas = good_ik_sols(sols)
    print(thetas)
        # thetas = thetas[0]
        # thetas = thetas[0]
    # return [thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]]


def main():
    pose_1 = [-0.66558183, -0.09470184, 0.0500, 180.0, 0.0, 0.0]

    t0 = time()
    # while i < 1000:

    m = solve_IK(pose_1)
    # print m
    t1 = time()
    print 'IK is found in %f seconds' % (t1 - t0)


if __name__ == "__main__":
    main()
