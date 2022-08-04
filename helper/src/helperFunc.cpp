#include "../helperFunc.h"


bool isStationary(const geometry_msgs::Twist this_vel, float maxSpeed, float maxTiltSpeed){
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

bool isFlat(const geometry_msgs::Pose this_pos, float maxTilt){
    return (this_pos.orientation.x <= maxTilt && this_pos.orientation.y <= maxTilt);
}

bool reachedLocation(const geometry_msgs::Pose this_pos, const geometry_msgs::PoseStamped desired_pos, float accuracyDistance){
    return( 
        pow( 
            pow(this_pos.position.x - desired_pos.pose.position.x, 2) + 
            pow(this_pos.position.y - desired_pos.pose.position.y, 2) + 
            pow(this_pos.position.z - desired_pos.pose.position.z, 2), .5) <= accuracyDistance );
}

bool reachedLocation(const geometry_msgs::Pose this_pos, const geometry_msgs::Point desired_pos, float accuracyDistance){
    return( 
        pow( 
            pow(this_pos.position.x - desired_pos.x, 2) + 
            pow(this_pos.position.y - desired_pos.y, 2) + 
            pow(this_pos.position.z - desired_pos.z, 2), .5) <= accuracyDistance );
}


mavros_msgs::PositionTarget pointInfoGenerator(const mav_trajectory_generation::Segment& s, double time){
    mavros_msgs::PositionTarget result;
    Eigen::VectorXd pos = s.evaluate(time, 0);
    Eigen::VectorXd vel = s.evaluate(time, 1);
    Eigen::VectorXd acc = s.evaluate(time, 2);

    result.header.stamp = ros::Time::now();
    result.header.frame_id = "map";

    result.coordinate_frame = 1;

    result.position.x = pos[0];
    result.position.y = pos[1];
    result.position.z = pos[2];

    result.velocity.x = vel[0];
    result.velocity.y = vel[1];
    result.velocity.z = vel[2];

    result.acceleration_or_force.x = acc[0];
    result.acceleration_or_force.y = acc[1];
    result.acceleration_or_force.z = acc[2];

    return result;
}