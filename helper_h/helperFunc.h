#ifndef HELPERFUNC_H
#define HELPERFUNC_H

/*  Includes    */
#include <ros/ros.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>


/*  Functions   */

/*Makes sures this_pos is not tilted more than maxTilt, degrees, in any axis*/
bool isFlat(const geometry_msgs::Pose this_pos, float maxTilt = 0);

/* Finds magnitude of this_vel values, x and y and z, checks its less than or equal to maxSpeed(m/s)*/
bool isStationary(const geometry_msgs::Twist this_vel, float maxSpeed = 0, float maxTiltSpeed = 0);

/*Finds distance from curr_pos to desired_pos and checks it's within a accuracyDistance*/
bool reachedLocation(const geometry_msgs::Pose this_pos, const geometry_msgs::PoseStamped desired_pos, float accuracyDistance);

/*  Classes   */

/*
SegmentStorage stores coefficents for position, velocity,
acceleration seperately for x,y,z axis. 
*/
class SegmentStorage{
    private:
        double* positionCoef; 
        double* velocityCoef;
        double* accelerationCoef;
    public:
        SegmentStorage(const mav_trajectory_generation::Segment& segment);
};

#endif