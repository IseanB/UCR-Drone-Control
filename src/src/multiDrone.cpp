#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // ros::Publisher drone_pub = nh.advertise<std::String>
    //         ("drone1", 50);
    while(ros::ok()){
        drone_pub.publish("Hello World...");
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
