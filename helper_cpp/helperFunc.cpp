#include "../helper_h/helperFunc.h"

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

SegmentStorage::SegmentStorage(const mav_trajectory_generation::Segment& segment){
    /*Transformation follows 1 Segment->3 Polynomial-> 3 VectorXd->  -> 10 double[]*/
    mav_trajectory_generation::Polynomial::Vector segmentPoly = segment.getPolynomialsRef();
    
    //VectorXd is a typedef of Matrix class
    // its a "typedef Matrix< double, Dynamic, 1 >"
    //not seeing segment function and polynomial funciton

    mav_trajectory_generation::Polynomial pos_poly_x = segmentPoly[0];
    Eigen::VectorXd position_x(10);

    // pos_x.getCoefficients(0);
    // Eigen::VectorXd position_y = segmentPoly[1].getCoefficients(0); // [1] = y-axis, 0 = position
    // Eigen::VectorXd position_z = segmentPoly[2].getCoefficients(0); // [2] = z-axis, 0 = position

    //.array() returns a ArrayWrapper<Derived>, .data() returns a Scalar pointer
    // for(unsigned i = 0; i < 10; ++i){
    //     std::cout << position_x[i] << std::endl;
    // }

    //position_x.array().data().begin();

}