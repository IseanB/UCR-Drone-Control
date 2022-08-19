#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::Publisher drone_pub = nh.advertise<std_msgs::String>
            ("drone1_info", 50);
    while(ros::ok()){
        std_msgs::String msg;
        msg.data = "Hellow world";
        drone_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
