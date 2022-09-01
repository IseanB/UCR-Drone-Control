#include <ros/ros.h>
#include "drone_control/dcontrol.h"
#include "drone_control/dresponse.h"

drone_control::dresponse d0_response;
drone_control::dresponse d1_response;
drone_control::dresponse d2_response;
drone_control::dresponse d3_response;


void storeInfo(const drone_control::dresponse::ConstPtr& msg);
void outputMenu();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;
    ros::Time last_request = ros::Time::now();
    ros::Rate rate(20.0);

    ros::Publisher drone1_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone0/cmds", 5);
    ros::Publisher drone2_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone1/cmds", 5);
    ros::Publisher drone3_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone2/cmds", 5);
    ros::Publisher drone4_cmd_pub = nh.advertise<drone_control::dcontrol>
            ("drone3/cmds", 5);
    
    ros::Subscriber d0_multi_sub = nh.subscribe<drone_control::dresponse>("drone0/info", 0, storeInfo);
    ros::Subscriber d1_multi_sub = nh.subscribe<drone_control::dresponse>("drone1/info", 0, storeInfo);
    ros::Subscriber d2_multi_sub = nh.subscribe<drone_control::dresponse>("drone2/info", 0, storeInfo);
    ros::Subscriber d3_multi_sub = nh.subscribe<drone_control::dresponse>("drone3/info", 0, storeInfo);
    
    drone_control::dcontrol msg;

    std::string input;
    char command;
    double x,y,z;
    outputMenu();
    std::cout << "Insert Command: ";
    while(ros::ok() && std::cin >> input){
        // Gets all neccassary info
        command = input[0];
        if(command == 'a' || command == 'n' || command == 'l'){
            std::cout << "Insert X Point: ";
            std::cin >> x;
            std::cout << "Insert Y Point: ";
            std::cin >> y;
            std::cout << "Insert Z Point: ";
            std::cin >> z;
        }
        // executes on information
        if(command == 'S'){
            msg.command.data = "SHUTOFF";
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: SHUTOFF    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 's'){
            msg.command.data = "STOP";
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: STOP    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'l'){
            msg.command.data = "LIFT";
            msg.target.x = x;
            msg.target.y = y;
            msg.target.z = z;
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: LIFT    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'L'){
            msg.command.data = "LAND";
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: LAND    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'a'){
            msg.command.data = "TRANSIT_ADD";
            msg.target.x = x;
            msg.target.y = y;
            msg.target.z = z;
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu(); 
            std::cout << "   EXECUTED: TRANSIT_ADD    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'n'){
            msg.command.data = "TRANSIT_NEW";
            msg.target.x = x;
            msg.target.y = y;
            msg.target.z = z;
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: TRANSIT_NEW    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'q'){
            msg.command.data = "LAND";
            std::cout << "\n\n\n\n\n\n---------------------------\n";
            std::cout << "   EXECUTING: QUIT    \n";
            std::cout << "---------------------------\n";
            drone1_cmd_pub.publish(msg);
            drone2_cmd_pub.publish(msg);
            drone3_cmd_pub.publish(msg);
            drone4_cmd_pub.publish(msg);
            while()
            std::cout << "Insert Command: ";
        }
        else{
            outputMenu();
            std::cout << "Error!!! Insert Valid Command:";
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void storeInfo(const drone_control::dresponse::ConstPtr& msg){
    d1_response = *msg;
}

void outputMenu(){
    std::cout << "\n\n\n\n\n\n\n\n\n\n";
    std::cout << "---------------------------\n";
    std::cout << "         Main Menu\n";
    std::cout << "---------------------------\n";
    std::cout << "Input Format: \n";
    std::cout << "(command) (x) (y) (z) \n";
    std::cout << "---------------------------\n";
    std::cout << "Input Commands:       Description:\n";
    std::cout << "  S                   Calls \"Shutoff\" command\n";
    std::cout << "  s                   Calls \"Stop\" command\n";
    std::cout << "  L                   Calls \"Land\" command\n";
    std::cout << "  l (x) (y) (z)       Calls \"Lift\" command. Insert \"0 0 0\" for default liftoff.\n";
    std::cout << "  a (x) (y) (z)       Calls \"Add Transit\" command\n";
    std::cout << "  n (x) (y) (z)       Calls \"New Transit\" command\n";
    std::cout << "  q                   Quit in controlling the drones\n";
    std::cout << "---------------------------\n";
}
