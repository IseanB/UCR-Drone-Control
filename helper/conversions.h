#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::PositionTarget segmentToPoint(const mav_trajectory_generation::Segment& s, double time);


#endif