#include <ros/ros.h>
#include "drone_control/dcontrol.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;
    ros::Time last_request = ros::Time::now();
    ros::Rate rate(20.0);

    ros::Publisher drone_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone1_cmds", 0);
    
    drone_control::dcontrol msg;
    msg.command.data = "LIFT";
    msg.target.z = 1;
    while(ros::ok()){
        if(ros::Time::now() - last_request > ros::Duration(10.0)){
            ROS_INFO("info sent");
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}