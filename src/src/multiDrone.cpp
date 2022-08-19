#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;
    ros::Time last_request = ros::Time::now();
    ros::Rate rate(20.0);

    ros::Publisher drone_cmd_pub = nh.advertise<std_msgs::String>
            ("drone_cmd", 0);

    ros::Publisher drone_pos_pub = nh.advertise<geometry_msgs::Point>
            ("drone1/target", 0);
    
    std_msgs::String msg;
    while(ros::ok()){
        if(ros::Time::now() - last_request > ros::Duration(10.0)){
            ROS_INFO("info sent");
            msg.data = "LIFT";
            drone_cmd_pub.publish(msg);
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

/*
LIFT
STOP
LAND
*/