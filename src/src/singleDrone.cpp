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
#include "drone_control/dcontrol.h"

enum PossiableState{
    GROUND_IDLE,
    LIFTING_OFF,
    HOVERING,
    IN_TRANSIT,
    LANDING,
    SHUTTING_DOWN // A drone is only momentarily in this state
};

//Must use commands to start,end. Add point

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_position;
geometry_msgs::PoseStamped hover_position;
geometry_msgs::Twist curr_velocity;
geometry_msgs::PoseStamped curr_target;
drone_control::dcontrol curr_message;

geometry_msgs::PoseStamped default_liftoff_pos;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;

PossiableState droneState;


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

/* Stores msgs from multi control node in last*/
void storeCommand(const drone_control::dcontrol::ConstPtr& inputMsg);

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
    std::string uavName = "drone" + static_cast<std::string>(argv[1]);
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

    // mutli control subscribers
    ros::Subscriber multi_sub = nh.subscribe<drone_control::dcontrol>
            (uavName + "_cmds", 0, storeCommand);

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
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(3))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(3))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO(uavName, " armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            if(curr_target.pose.position.x == 0 && curr_target.pose.position.y == 0 && curr_target.pose.position.z == 0){
                local_pos_pub.publish(default_liftoff_pos);
                if((reachedLocation(curr_position, default_liftoff_pos, .1)) && (isStationary(curr_velocity, .05))){
                    hover_position = default_liftoff_pos;
                    droneState = HOVERING;
                    ROS(uavName, " Hovering");
                }
            }
            else{
                local_pos_pub.publish(curr_target);
                if((reachedLocation(curr_position, curr_target, .1)) && (isStationary(curr_velocity, .05))){
                    hover_position = curr_target;
                    droneState = HOVERING;
                    ROS(uavName, " Hovering");
                }
            }

        }else if(droneState == HOVERING){
            local_pos_pub.publish(hover_position);
        }else if(droneState == IN_TRANSIT){
            if(ros::Time::now() - last_request > ros::Duration(30.0)){
                ROS_INFO("In Transit...");
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
    curr_position.pose.position.x = 0;
    curr_position.pose.position.y = 0;
    curr_position.pose.position.z = 0;
    curr_position.pose.orientation.x = 0;
    curr_position.pose.orientation.y = 0;
    curr_position.pose.orientation.z = 0;
    curr_position.pose.orientation.w = 0;

    curr_velocity.linear.x = 0;
    curr_velocity.linear.y = 0;
    curr_velocity.linear.z = 0;
    curr_velocity.angular.x = 0;
    curr_velocity.angular.y = 0;
    curr_velocity.angular.z = 0;

    curr_target.pose.position.x = 0;
    curr_target.pose.position.y = 0;
    curr_target.pose.position.z = 0;

    hover_position.pose.position.x = 0;
    hover_position.pose.position.y = 0;
    hover_position.pose.position.z = 0;

    droneState = GROUND_IDLE;

    default_liftoff_pos.pose.position.x = curr_position.pose.position.x;
    default_liftoff_pos.pose.position.y = curr_position.pose.position.y;
    default_liftoff_pos.pose.position.z = 2;

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void updatePose(const geometry_msgs::PoseStamped::ConstPtr& inputPose){
    curr_position = *inputPose;
    // curr_position.pose.position.x = (inputPose->pose).position.x;
    // curr_position.pose.position.y = (inputPose->pose).position.y;
    // curr_position.pose.position.z = (inputPose->pose).position.z;
    // curr_position.pose.orientation.x = (inputPose->pose).orientation.x;
    // curr_position.pose.orientation.y = (inputPose->pose).orientation.y;
    // curr_position.pose.orientation.z = (inputPose->pose).orientation.z;
    // curr_position.pose.orientation.w = (inputPose->pose).orientation.w;
}

void updateVel(const geometry_msgs::TwistStamped::ConstPtr& inputPose){
    curr_velocity.linear.x = inputPose->twist.linear.x;
    curr_velocity.linear.y = inputPose->twist.linear.y;
    curr_velocity.linear.z = inputPose->twist.linear.z;
    curr_velocity.angular.x = inputPose->twist.angular.x;
    curr_velocity.angular.y = inputPose->twist.angular.y;
    curr_velocity.angular.z = inputPose->twist.angular.z;
}

void storeCommand(const drone_control::dcontrol::ConstPtr& inputMsg){
    curr_message = *inputMsg;
    //handles input target
    if(inputMsg->target.x != curr_target.pose.position.x || 
       inputMsg->target.y != curr_target.pose.position.y ||
       inputMsg->target.z != curr_target.pose.position.z){
        curr_target.pose.position.x = inputMsg->target.x;  
        curr_target.pose.position.y = inputMsg->target.y;
        curr_target.pose.position.z = inputMsg->target.z;
    }
    
    //handles input command
    std::string inputCmd = inputMsg->command.data;
    if(inputCmd == "E_STOP"){
        droneState = SHUTTING_DOWN;
    }
    else if(inputCmd == "LIFT"){
        if(droneState == GROUND_IDLE){
            droneState = LIFTING_OFF;
        }
        else{
            ROS_INFO("LIFT Error: Drone is off the ground.");
        }
    }
    else if(inputCmd == "STOP"){
        if(droneState == GROUND_IDLE){
            ROS_INFO("STOP Error: Drone is already stationary.");
        }
        else;
            hover_position = curr_position;
            droneState = HOVERING;
    }
    else if(inputCmd == "LAND"){
        if(droneState == LANDING || droneState == GROUND_IDLE){
            ROS_INFO("LAND Error: Drone is already landing/landed.");
        }
        else;
            droneState = LANDING;
    }
    else{
        ROS_INFO("Error: Invalid Command.");
    }
}

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