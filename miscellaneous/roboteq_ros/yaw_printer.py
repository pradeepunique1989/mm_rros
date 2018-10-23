#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

def callback(data):
    [r,p,yaw] = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
    print 'x:\t',data.pose.pose.position.x
    print 'y:\t',data.pose.pose.position.y
    print 'yaw:\t',yaw

def yaw_plotter():

    rospy.init_node('yaw_plotter_node', anonymous=True)
    rospy.Subscriber("odom_test", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    yaw_plotter()

