#include <ros/ros.h>
#include "drone_control/dcontrol.h"
#include "drone_control/dresponse.h"

const uint16_t TOTAL_DRONES = 4; 

drone_control::dresponse drone_responses[TOTAL_DRONES]; // stores the responses for each drone
ros::Publisher drone_cmd_pub[TOTAL_DRONES]; // stores all of the publishers, commands, for each drone
ros::Subscriber drone_info_sub[TOTAL_DRONES]; // stores all of the publishers, commands, for each drone


void setup(ros::NodeHandle& nodehandler);
void outputMenu();
void storeInfo(const drone_control::dresponse::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_control_node");
    ros::NodeHandle nh;
    setup(nh);
    ros::Time last_request = ros::Time::now();
    ros::Rate rate(20.0);
    
    drone_control::dcontrol msg;

    std::string input;
    char command;
    double x,y,z;
    bool done = false;
    outputMenu();
    std::cout << "Insert Command: ";
    while(ros::ok() && std::cin >> input && !done){
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
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: SHUTOFF    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
            done = true;
        }
        else if(command == 's'){
            msg.command.data = "STOP";
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
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
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: LIFT    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'L'){
            msg.command.data = "LAND";
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
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
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
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
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
            outputMenu();
            std::cout << "   EXECUTED: TRANSIT_NEW    \n";
            std::cout << "---------------------------\n";
            std::cout << "Insert Command: ";
        }
        else if(command == 'q'){
            // first land the drones
            msg.command.data = "LAND";
            std::cout << "\n\n\n\n\n\n---------------------------\n";
            std::cout << "   EXECUTING: QUIT    \n";
            std::cout << "---------------------------\n";
            drone_cmd_pub[0].publish(msg);
            drone_cmd_pub[1].publish(msg);
            drone_cmd_pub[2].publish(msg);
            drone_cmd_pub[3].publish(msg);
            ros::spinOnce();
            rate.sleep();

            // second shutoff the drones
            msg.command.data = "SHUTOFF";
            bool landed = false;
            while(!landed){
                int numDronesLanded = 0;
                for(drone_control::dresponse response : drone_responses){
                    if(response.state == 0){
                        drone_cmd_pub[response.droneid].publish(msg);
                        ++numDronesLanded;
                    }
                }
                if(numDronesLanded == TOTAL_DRONES)
                    landed = true;
                    done = true;
            }
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

void setup(ros::NodeHandle& nodehandler){
    int droneCounter = 0;
    while(droneCounter != TOTAL_DRONES){
        drone_cmd_pub[droneCounter] = nodehandler.advertise<drone_control::dcontrol>("drone" + std::to_string(droneCounter) + "/cmds", 5);
        drone_info_sub[droneCounter] = nodehandler.subscribe<drone_control::dresponse>("drone" + std::to_string(droneCounter) + "/info", 0, storeInfo);
        ++droneCounter;
    }
}

void storeInfo(const drone_control::dresponse::ConstPtr& msg){// assumes droneid <= TOTAL_DRONES
    drone_responses[msg->droneid] = *msg;
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
