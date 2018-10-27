/**
 * File              : ../mm_simulation_control/src/VrepInterface.cpp
 * Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
 * Date              : 26.10.2018
 * Last Modified Date: 26.10.2018
 * Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
 */
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include "mm_simulation_control/VrepInterface.hpp"
#include <ros/console.h>

// VREP remote api
extern "C" {
    #include "vrep_api/extApi.h"
    #include "vrep_api/v_repConst.h"
}

VrepInterface::VrepInterface(ros::NodeHandle& n) :
        clientID_(-1),
        sync_(false) {
    // Connect to V-REP via remote api
    ROS_INFO("Waiting for valid time. Is V-REP running?");
    ros::Time::waitForValid();
    while(clientID_ == -1 && ros::ok()) {
        clientID_ = simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
        if (clientID_ == -1) {
            ROS_ERROR_STREAM("Couldn't connect to V-REP remote api");
        }
    }
    ROS_INFO("Connected to V-REP");

    // Enable synchronous mode if needed
    if (sync_) {
        simxSynchronous(clientID_, true);
        simxSynchronousTrigger(clientID_);
        ROS_INFO("Enabled V-REP sync mode");
    }

    // Initialise ROS subscribers & publishers
    // jointPub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    // if (torqueMode_) {
    //     torqueSub_ = n.subscribe("target_torques", 1,
    //             &VrepInterface::torqueCallback, this);
    // }
    //
    // publishWorkerTimer_ = n.createWallTimer(ros::WallDuration(1.0 / feedbackRate_),
    //         &VrepInterface::publishWorker, this);
    ROS_INFO("VrepInterface initialised");

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "vrep_interface");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    VrepInterface vrep(n);
    ros::waitForShutdown();
    return 0;
}
