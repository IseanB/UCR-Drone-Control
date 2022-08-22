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
            ("drone1/cmds", 0);
    ros::Subscriber drone_info_sub = nh.subscribe<drone_control::dresponse>
            ("drone1/info", 0, storeInfo);
    
    drone_control::dcontrol msg;
    bool lift = false;
    bool transit = false;
    bool transit2 = false;
    bool transit3 = false;
    bool transit4 = false;
    bool transit5 = false;
    bool transit6 = false;
    bool transit7 = false;
    // bool transit2 = false;
    while(ros::ok() && !lift){
        if(ros::Time::now() - last_request >= ros::Duration(1.0)){
            ROS_INFO("LIFT info sent");
            msg.command.data = "LIFT";
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            lift = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok() && !transit7){
        if(!transit && ros::Time::now() - last_request >= ros::Duration(20.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_ADD";
            msg.target.x = 1;
            msg.target.y = 1;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit = true;
        }
        else if(!transit2 && transit && ros::Time::now() - last_request >= ros::Duration(15.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_ADD";
            msg.target.x = -1;
            msg.target.y = -2;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit2 = true;
        }
        else if(!transit3 && transit2 && ros::Time::now() - last_request >= ros::Duration(1.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_ADD";
            msg.target.x = 0;
            msg.target.y = 0;
            msg.target.z = 1.5;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit3 = true;
        }
        else if(!transit4 && transit3 && ros::Time::now() - last_request >= ros::Duration(3.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_NEW";
            msg.target.x = -1;
            msg.target.y = 2;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit4 = true;
        }
        else if(!transit5 && transit4 && ros::Time::now() - last_request >= ros::Duration(3.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_NEW";
            msg.target.x = 1;
            msg.target.y = -2;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit5 = true;
        }
        else if(!transit6 && transit5 && ros::Time::now() - last_request >= ros::Duration(3.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_ADD";
            msg.target.x = 3;
            msg.target.y = -2;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit6 = true;
        }
        else if(!transit7 && transit6 && ros::Time::now() - last_request >= ros::Duration(10.0)){
            ROS_INFO(" TRANSIT info sent");
            msg.command.data = "TRANSIT_NEW";
            msg.target.x = 4;
            msg.target.y = 1;
            msg.target.z = 1;
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
            transit7 = true;
            ROS_INFO("DONE");
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


void storeInfo(const drone_control::dresponse::ConstPtr& msg){
    d1_response = *msg;
}