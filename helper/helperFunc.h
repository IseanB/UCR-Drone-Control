#ifndef HELPERFUNC_H
#define HELPERFUNC_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

/*Makes sures this_pos is not tilted more than maxTilt, degrees, in any axis*/
bool isFlat(const geometry_msgs::Pose this_pos, float maxTilt = 0);

/* Finds magnitude of this_vel values, x and y and z, checks its less than or equal to maxSpeed(m/s)*/
bool isStationary(const geometry_msgs::Twist this_vel, float maxSpeed = 0, float maxTiltSpeed = 0);

/*Finds distance from curr_pos to desired_pos and checks it's within a accuracyDistance*/
bool reachedLocation(const geometry_msgs::Pose this_pos, const geometry_msgs::PoseStamped desired_pos, float accuracyDistance);

#endif