/*  Custom files  */
#include "../../helper/conversions.h"
#include "../../helper/computations.h"

/*  Libraries   */
#include <ros/ros.h>

/*  Data Types  */
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>

enum PossiableState{
    GROUND_IDLE,
    LIFTING_OFF,
    IN_TRANSIT,
    HOVERING,
    LANDING,
    SHUTTING_DOWN // A drone is only momentarily in this state
};

mavros_msgs::State current_state;
geometry_msgs::Pose curr_position;
geometry_msgs::Twist curr_velocity;
PossiableState droneState = GROUND_IDLE;

/* Assumes drone is on ground. Initializes curr_position & curr_velocity values to 0 */
void setup();

/* Used for initiating contact with drone and setting it to OFFBOARD mode */
void state_cb(const mavros_msgs::State::ConstPtr& msg);

/* Updates curr_position values, position and orientation, with inputted pose */
void updatePose(const geometry_msgs::PoseStamped::ConstPtr& inputPose);

/* Updates curr_velocity values, linear and angular, with inputted pose */
void updateVel(const geometry_msgs::TwistStamped::ConstPtr& inputPose);

/* Prints crutial information about the inputted trajectory */
// void printTrajInfo(const mav_trajectory_generation::Segment::Vector& allSegments);

/* Stores msgs from multi control node IN */
void storeCommand(const std_msgs::String::ConstPtr& inputMsg);

int main(int argc, char **argv)
{
    // initalizing node
    std::cout << "Initalizing Drone " + static_cast<std::string>(argv[1]) + " ..." << std::endl;
    if(argc != 2){
        throw;
    }

    ros::init(argc, argv, "drone" + static_cast<std::string>(argv[1]));
    ros::NodeHandle nh;
    
    setup();
    std::string dPrefix = "uav" + static_cast<std::string>(argv[1]) + "/";
    ros::Time last_request = ros::Time::now(); // used for periodic messaging

    // all topics
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (dPrefix +"mavros/state", 0, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (dPrefix +"mavros/local_position/pose", 0, updatePose);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            (dPrefix +"mavros/local_position/velocity", 0, updateVel);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (dPrefix +"mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (dPrefix +"mavros/set_mode");
    ros::ServiceClient end_flight_client = nh.serviceClient<mavros_msgs::CommandLong>
            (dPrefix +"mavros/cmd/command");        

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (dPrefix +"mavros/setpoint_position/local", 0);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            (dPrefix +"mavros/setpoint_velocity/cmd_vel_unstamped", 0);
    ros::Publisher mav_pub = nh.advertise<mavros_msgs::PositionTarget>
            (dPrefix +"mavros/setpoint_raw/local", 20);

    // subscribes to mutli_control
    ros::Subscriber command_sub = nh.subscribe<std_msgs::String>
            ("drones_cmd", 0, storeCommand);
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::Point>
            ("drone" + static_cast<std::string>(argv[1]) +"/target", 0, store);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Drone " + static_cast<std::string>(argv[1]) + " Initialized!\n";

    // state based control
    while(ros::ok() && droneState != SHUTTING_DOWN){
        if(droneState == GROUND_IDLE){
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("Ground Idling...");
                last_request = ros::Time::now();
            }
        }else if(droneState == LIFTING_OFF){
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("Lifting Off...");
                last_request = ros::Time::now();
            }
        }else if(droneState == IN_TRANSIT){
            if(ros::Time::now() - last_request > ros::Duration(30.0)){
                ROS_INFO("In Transit...");
                last_request = ros::Time::now();
            }
        }else if(droneState == HOVERING){
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("Hovering...");
                last_request = ros::Time::now();
            }
        }else if(droneState == LANDING){
            if(ros::Time::now() - last_request > ros::Duration(2.0)){
                ROS_INFO("Landing...");
                last_request = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}
void setup(){
    curr_position.position.x = 0;
    curr_position.position.y = 0;
    curr_position.position.z = 0;
    curr_position.orientation.x = 0;
    curr_position.orientation.y = 0;
    curr_position.orientation.z = 0;
    curr_position.orientation.w = 0;

    curr_velocity.linear.x = 0;
    curr_velocity.linear.y = 0;
    curr_velocity.linear.z = 0;
    curr_velocity.angular.x = 0;
    curr_velocity.angular.y = 0;
    curr_velocity.angular.z = 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void updatePose(const geometry_msgs::PoseStamped::ConstPtr& inputPose){
    curr_position.position.x = (inputPose->pose).position.x;
    curr_position.position.y = (inputPose->pose).position.y;
    curr_position.position.z = (inputPose->pose).position.z;
    curr_position.orientation.x = (inputPose->pose).orientation.x;
    curr_position.orientation.y = (inputPose->pose).orientation.y;
    curr_position.orientation.z = (inputPose->pose).orientation.z;
    curr_position.orientation.w = (inputPose->pose).orientation.w;
}

void updateVel(const geometry_msgs::TwistStamped::ConstPtr& inputPose){
    curr_velocity.linear.x = inputPose->twist.linear.x;
    curr_velocity.linear.y = inputPose->twist.linear.y;
    curr_velocity.linear.z = inputPose->twist.linear.z;
    curr_velocity.angular.x = inputPose->twist.angular.x;
    curr_velocity.angular.y = inputPose->twist.angular.y;
    curr_velocity.angular.z = inputPose->twist.angular.z;
}

void storeCommand(const std_msgs::String::ConstPtr& inputMsg){
    if(inputMsg->data == "LIFT"){
        if(droneState == GROUND_IDLE);
            droneState = LIFTING_OFF;
        else;
            ROS_INFO("LIFT Error: Drone is off the ground.");
    }
    else if(inputMsg->data == "STOP"){
        if(droneState == GROUND_IDLE || droneState == HOVERING);
            ROS_INFO("STOP Error: Drone is already stationary.");
        else;
            droneState = HOVERING;
    }
    else if(inputMsg->data == "LAND"){
        if(droneState == LANDING || droneState == GROUND_IDLE);
            ROS_INFO("LAND Error: Drone is already landing/landed.");
        else;
            droneState = LANDING;
    }
    else;
        ROS_INFO("Error: Invalid Command.");
}
// enum PossiableState{
//     GROUND_IDLE,
//     LIFTING_OFF,
//     IN_TRANSIT,
//     HOVERING,
//     LANDING,
//     SHUTTING_DOWN
// };

// void printTrajInfo(const mav_trajectory_generation::Segment::Vector& allSegments){
//     mav_trajectory_generation::Trajectory trajectory;
//     trajectory.setSegments(allSegments);// dont use addSegments, uncessary calculations
//     std::cout << "Trajectory Properties:\n" << std::endl;
//     std::cout << "Number of Dimensions :  "<<trajectory.D() << std::endl;
//     std::cout << "Polynomial Order of Optimizination Function :  "<<trajectory.N() << std::endl;
//     std::cout << "Number of Segments :  " << trajectory.K() << std::endl << std::endl;
//     std::cout << "Sample Optimized Function Information: \n\n"; 
//     /*Line below prints optimized functions;x,y,z axis; for first segement in position*/
//     printSegment(std::cout, allSegments.at(0), 0);
//     /*Line below prints optimized functions;x,y,z axis; for first segement in veloity*/
//     printSegment(std::cout, allSegments.at(0), 1);
//     /*Line below prints optimized functions;x,y,z axis; for first segement in acceleration*/
//     printSegment(std::cout, allSegments.at(0), 2);
// }