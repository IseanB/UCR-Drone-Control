#include "../computations.h"

bool isStationary(const geometry_msgs::Twist& this_vel, float maxSpeed, float maxTiltSpeed){
    return( pow(
        pow(this_vel.linear.x, 2) + 
        pow(this_vel.linear.y, 2) + 
        pow(this_vel.linear.z, 2), .5) <= maxSpeed && 
        pow(
            pow(this_vel.angular.x, 2) + 
            pow(this_vel.angular.y, 2) + 
            pow(this_vel.angular.z, 2), .5) <= maxTiltSpeed
         );
}

bool isFlat(const geometry_msgs::Pose& this_pos, float maxTilt){
    return (abs(this_pos.orientation.x) <= abs(maxTilt) && abs(this_pos.orientation.y) <= abs(maxTilt));
}

bool reachedLocation(const geometry_msgs::Pose& this_pos, const geometry_msgs::PoseStamped& desired_pos, float accuracyDistance){
    return( 
        pow( 
            pow(this_pos.position.x - desired_pos.pose.position.x, 2) + 
            pow(this_pos.position.y - desired_pos.pose.position.y, 2) + 
            pow(this_pos.position.z - desired_pos.pose.position.z, 2), .5) <= abs(accuracyDistance) );
}


bool reachedLocation(const geometry_msgs::Pose& this_pos, const mavros_msgs::PositionTarget& desired_pos, float accuracyDistance){
    return( 
        pow( 
            pow(this_pos.position.x - desired_pos.position.x, 2) + 
            pow(this_pos.position.y - desired_pos.position.y, 2) + 
            pow(this_pos.position.z - desired_pos.position.z, 2), .5) <= abs(accuracyDistance) );
}