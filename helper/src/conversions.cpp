#include "../conversions.h"

mavros_msgs::PositionTarget segmentToPoint(const mav_trajectory_generation::Segment& s, float time){
    mavros_msgs::PositionTarget result;
    Eigen::VectorXd pos = s.evaluate(time, 0);
    Eigen::VectorXd vel = s.evaluate(time, 1);
    Eigen::VectorXd acc = s.evaluate(time, 2);

    result.header.stamp = ros::Time::now();
    result.header.frame_id = "map";
    result.type_mask = 1024+2048;

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