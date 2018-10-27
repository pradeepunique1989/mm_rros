/**
 * File              : include/mm_simulation_control/VrepInterface.hpp
 * Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
 * Date              : 26.10.2018
 * Last Modified Date: 26.10.2018
 * Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
 */
/** @file VrepInterface.hpp
 * Acts as the interface between ROS and V-REP for controlling the Jaco arm.
 * Implements ROS's control_msgs::FollowJointTrajectoryAction action interface,
 * and uses V-REP's C++ api as this is faster than V-REP's ROS api.
 */
#ifndef MM_SIMULATION_CONTROL_VREP_INTERFACE_HPP_
#define MM_SIMULATION_CONTROL_VREP_INTERFACE_HPP_

#include <unordered_map>
#include <string>
#include <iostream>
#include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

class VrepInterface {

public:
    VrepInterface(ros::NodeHandle& n);
private:
    /** VREP connection id */
    int clientID_;
    /** Whether to use synchronous mode with V-REP */
    bool sync_;
};

#endif 
