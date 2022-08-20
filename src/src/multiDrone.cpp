#include <ros/ros.h>
#include "drone_control/dcontrol.h"
#include "drone_control/dresponse.h"

drone_control::dresponse d1_response;

void storeInfo(const drone_control::dresponse::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;
    ros::Time last_request = ros::Time::now();
    ros::Rate rate(20.0);

    ros::Publisher drone_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone1_cmds", 0);
    ros::Subscriber drone_info_sub = nh.subscribe<drone_control::dresponse>
            ("drone1_cmds", 0, storeInfo);
    
    drone_control::dcontrol msg;
    bool done = false;
    while(ros::ok()){
        while(ros::Time::now() - last_request < ros::Duration(10.0)){
            if(ros::Time::now() - last_request >= ros::Duration(10.0)){
                ROS_INFO("info sent");
                msg.command.data = "LIFT";
                msg.target.z = 1;
                drone_cmd_pub.publish(msg);
                last_request = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }
        while(ros::Time::now() - last_request > ros::Duration(10.0)){
            if(ros::Time::now() - last_request >= ros::Duration(10.0)){
                msg.command.data = "IN_TRANSIT";
                last_request = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}


void storeInfo(const drone_control::dresponse::ConstPtr& msg){
    d1_response = *msg;
}