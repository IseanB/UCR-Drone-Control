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
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "drone_control/dcontrol.h"
#include "drone_control/dresponse.h"
#include <queue>

enum PossiableState{
    GROUND_IDLE,
    LIFTING_OFF,
    HOVERING,
    IN_TRANSIT,
    LANDING,
    SHUTTING_DOWN // A drone is only momentarily in this state
};

struct TrajectoryGroupedInfo{// Groups essential information to reduce # of dereferences
    mav_trajectory_generation::Segment segmentThis;
    geometry_msgs::PoseStamped startPoint;
    geometry_msgs::PoseStamped endPoint;
    float trajectoryTime;
    TrajectoryGroupedInfo(mav_trajectory_generation::Segment inputSeg, geometry_msgs::PoseStamped inputStart, geometry_msgs::PoseStamped inputEnd, float inputTime) : segmentThis(inputSeg), startPoint(inputStart), endPoint(inputEnd), trajectoryTime(abs(inputTime)){};
};

// highly dynamic data
mavros_msgs::State current_state;
geometry_msgs::Twist curr_velocity;
geometry_msgs::PoseWithCovariance curr_position;

// moderately dynamic data
PossiableState droneState;
geometry_msgs::Twist updatingVel;
drone_control::dcontrol curr_message;
geometry_msgs::PoseStamped hover_position;

// static data
std::string droneName;
std::string uavPrefix;
ros::Publisher local_pos_pub;
ros::Publisher multi_info_pub;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
drone_control::dresponse last_response;

// dynamic trajectory data
std::queue<TrajectoryGroupedInfo *> curr_trajectories;
float currTime;
float currTrajTime;
const float timeStepSize = .04;
mavros_msgs::PositionTarget curr_target;


void setup(ros::NodeHandle &nodehandler, std::string droneNum);

/* Used for initiating contact with drone and setting it to OFFBOARD/ARMED mode */
void state_cb(const mavros_msgs::State::ConstPtr &msg);

/* Updates curr_position, position and orientation, with inputted pose */
void updatePose(const nav_msgs::Odometry::ConstPtr &inputPose);

/* Updates curr_velocity, linear and angular, with inputted pose */
void updateVel(const geometry_msgs::TwistStamped::ConstPtr &inputPose);

/* Stores msgs from multi_control_node*/
void interpretCommand(const drone_control::dcontrol::ConstPtr &inputMsg);

/*Generates a trajectory to follow, using mav_trajectory_generation package*/
mav_trajectory_generation::Segment generateTraj(int bx, int by, int bz, int ex, int ey, int ez);

int main(int argc, char **argv){
    std::cout << "Initalizing Drone " + static_cast<std::string>(argv[1]) + " ..." << std::endl;
    if (argc != 2){ // exits program if no drone# was inputted
        throw;
    }

    ros::init(argc, argv, "drone" + static_cast<std::string>(argv[1]));
    ros::NodeHandle nh;
    setup(nh, static_cast<std::string>(argv[1]));

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uavPrefix + "mavros/state", 0, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>(uavPrefix + "mavros/global_position/local", 0, updatePose);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(uavPrefix + "mavros/global_position/raw/gps_vel", 0, updateVel);
    ros::Subscriber multi_cmd_sub = nh.subscribe<drone_control::dcontrol>(droneName + "/cmds", 0, interpretCommand);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uavPrefix + "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uavPrefix + "mavros/set_mode");
    ros::ServiceClient end_flight_client = nh.serviceClient<mavros_msgs::CommandLong>(uavPrefix + "mavros/cmd/command");

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(uavPrefix + "mavros/setpoint_velocity/cmd_vel_unstamped", 0);
    ros::Publisher mav_pub = nh.advertise<mavros_msgs::PositionTarget>(uavPrefix + "mavros/setpoint_raw/local", 0);
    // ros::Publisher local_pos_pub (global)
    // ros::Publisher  multi_info_pub (global)

    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now(); // used for periodic messaging

    // wait for FCU connection
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Sends a few points necessary for initialization 
    for (unsigned i = 0; i < 20; ++i){
        local_pos_pub.publish(hover_position);
    }
    std::cout << "Drone " + static_cast<std::string>(argv[1]) + " Initialized!\n";

    // state based control
    while(ros::ok() && droneState != SHUTTING_DOWN){
        if (droneState == GROUND_IDLE){
            if (ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("Ground Idling...");
                last_request = ros::Time::now();
            }
        }
        else if (droneState == LIFTING_OFF){
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(1.5)))
            {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("OFFBOARD ENABLED");
                }
                last_request = ros::Time::now();
            }
            else{
                if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.5))){
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("ARMED ENABLED");
                    }
                    last_request = ros::Time::now();
                }
            }
            local_pos_pub.publish(hover_position);
            if ((reachedLocation(curr_position.pose, hover_position, .2)) && (isStationary(curr_velocity, .07))){
                curr_target.position = hover_position.pose.position;
                droneState = HOVERING;
                currTrajTime = 0;
                last_response.response.data = "REACHED";
                multi_info_pub.publish(last_response);
            }
            if (ros::Time::now() - last_request > ros::Duration(10.0)){
                ROS_INFO("Lifting Offf...");
                last_request = ros::Time::now();
            }
        }
        else if (droneState == HOVERING){ 
        // update hover_position before transiting to HOVERING
            if (ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("HOVERING...");
                last_request = ros::Time::now();
            }
            local_pos_pub.publish(hover_position);
        }
        else if (droneState == IN_TRANSIT){
            if (curr_trajectories.size() == 0){ // no traj planned
                hover_position.pose.position = curr_target.position;
                currTrajTime = 0;
                currTime = 0;
                droneState = HOVERING;
            }
            else{ // at least one traj planned
                TrajectoryGroupedInfo *first_trajectory;
                if (currTrajTime == 0){ // trajectory not loaded
                    first_trajectory = curr_trajectories.front();
                    currTrajTime = first_trajectory->trajectoryTime;
                    currTime = 0;
                    curr_target = segmentToPoint(first_trajectory->segmentThis, currTime);
                    mav_pub.publish(curr_target);
                }
                else{ // trajectory loaded
                    if (reachedLocation(curr_position.pose, curr_target, .35)){ // if near end of curr_target
                        currTime += timeStepSize;
                        if (currTime > currTrajTime){
                            currTime = currTrajTime;
                        }
                        curr_target = segmentToPoint(first_trajectory->segmentThis, currTime-.001); // Make sures no invalid time is inputted
                        mav_pub.publish(curr_target);
                        if (currTime == currTrajTime){// at end of trajectory
                            curr_target.position = first_trajectory->endPoint.pose.position;
                            curr_trajectories.pop();
                            delete first_trajectory;
                            currTrajTime = 0;
                            last_response.response.data = "REACHED";
                            multi_info_pub.publish(last_response);
                        }
                    }
                    else{
                        mav_pub.publish(curr_target);
                    }
                }
            }
            if (ros::Time::now() - last_request > ros::Duration(10.0)){
                ROS_INFO("In Transit...");
                last_request = ros::Time::now();
            }
        }
        else if (droneState == LANDING){
            if(curr_position.pose.position.z > curr_target.position.z * .8){//parabolic curve 1
                updatingVel.linear.z = (-.3) * pow(abs(curr_position.pose.position.z-curr_target.position.z) + .01, 2);
            }
            else if(curr_position.pose.position.z > curr_target.position.z * .4){//parabolic curve 2(linear)
                updatingVel.linear.z = (-.3);
                local_vel_pub.publish(updatingVel);
            }
            else{//parabolic curve 3
                updatingVel.linear.z = (-.3) * pow(curr_position.pose.position.z, 2);
            }

            if((isStationary(curr_velocity, .07) && ( curr_position.pose.position.z <= .2)) || ( curr_position.pose.position.z <= .05) ){
                droneState = SHUTTING_DOWN;
            }
            if (ros::Time::now() - last_request > ros::Duration(2.0)){
                ROS_INFO("Landing...");
                last_request = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Sends shutoff command
    mavros_msgs::CommandLong endflight_call;
    endflight_call.request.broadcast = false;
    endflight_call.request.command = 400;
    endflight_call.request.param2 = 21196.0;
    if( end_flight_client.call(endflight_call) &&
        end_flight_client.waitForExistence()){
        ROS_INFO("Vehicle has shutdown");
    }

    return 0;
}

void setup(ros::NodeHandle &nodehandler, std::string droneNum){
    droneName = "drone" + droneNum;
    uavPrefix = "uav" + droneNum + "/";
    currTrajTime = 0;
    currTime = 0;

    hover_position.pose.position.x = curr_position.pose.position.x;
    hover_position.pose.position.y = curr_position.pose.position.y;
    hover_position.pose.position.z = 2; // sets the default liftoff position to 2m

    curr_target.position.x = curr_position.pose.position.x;
    curr_target.position.y = curr_position.pose.position.y;
    curr_target.position.z = 2;

    multi_info_pub = nodehandler.advertise<drone_control::dresponse>(droneName + "/info", 0);
    local_pos_pub = nodehandler.advertise<geometry_msgs::PoseStamped>(uavPrefix + "mavros/setpoint_position/local", 20);
    last_response.response.data = "NULL";
    offb_set_mode.request.custom_mode = "OFFBOARD"; // offboard command
    arm_cmd.request.value = true; // arming command
    droneState = GROUND_IDLE;

}

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

void updatePose(const nav_msgs::Odometry::ConstPtr &inputPose){
    curr_position = inputPose->pose;
}

void updateVel(const geometry_msgs::TwistStamped::ConstPtr &inputPose){
    curr_velocity = inputPose->twist;
}

mav_trajectory_generation::Segment generateTraj(int bx, int by, int bz, int ex, int ey, int ez){
    Eigen::Vector3d startPoint(bx, by, bz);
    Eigen::Vector3d endPoint(ex, ey, ez);

    mav_trajectory_generation::Segment::Vector segments;
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), end(dimension);

    start.makeStartOrEnd(startPoint, derivative_to_optimize);
    vertices.push_back(start);

    end.makeStartOrEnd(endPoint, derivative_to_optimize);
    vertices.push_back(end);

    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    const int N = 10; // needs to be at least 10 for min snap, 8 for min jerk
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    opt.getSegments(&segments); // fills vector of segment with optimized equations.
    return mav_trajectory_generation::Segment(segments[0]); // only one segment is generated, start -> end. Only that is returned.
}

void interpretCommand(const drone_control::dcontrol::ConstPtr &inputMsg){
    ros::Time last_request = ros::Time::now();
    last_response.response.data = "RECEIVED";
    curr_message = *inputMsg;

    std::string inputCmd = inputMsg->command.data;
    if (inputCmd == "SHUTOFF"){
        droneState = SHUTTING_DOWN;
    }
    else if (inputCmd == "LIFT"){
        if (droneState == GROUND_IDLE){
            if (inputMsg->target.z != 0){// if given a point, update default liftoff pos
                hover_position.pose.position = inputMsg->target;
            }
            curr_target.position = hover_position.pose.position;
            droneState = LIFTING_OFF;
        }
        else{
            ROS_INFO("LIFT Error: Drone is off the ground.");
            last_response.response.data = "ERROR";
        }
    }
    else if (inputCmd == "STOP"){
        if (droneState == GROUND_IDLE){
            ROS_INFO("STOP Error: Drone is already stationary.");
            last_response.response.data = "ERROR";
        }
        else{
            TrajectoryGroupedInfo* first_trajectory;
            while(curr_trajectories.size() != 0){
                first_trajectory = curr_trajectories.front();
                curr_trajectories.pop();
                delete first_trajectory;
            }

            hover_position.pose = curr_position.pose;
            curr_target.position = hover_position.pose.position;
            droneState = HOVERING;
        }
    }
    else if (inputCmd == "LAND"){
        if (droneState == LANDING || droneState == GROUND_IDLE){
            ROS_INFO("LAND Error: Drone is already landing/landed.");
            last_response.response.data = "ERROR";
        }
        else{
            // updates curr_target for parabloic blend
            curr_target.position.z = curr_position.pose.position.z;
            // makes sure no movement on xy plane
            updatingVel.linear.x = 0;
            updatingVel.linear.y = 0;
            droneState = LANDING;
        }
    }
    else if(inputCmd == "CHECK"){}// Will return last response at the bottom.
    else if (inputCmd == "TRANSIT_ADD" || inputCmd == "TRANSIT_NEW"){
        if (droneState == GROUND_IDLE){
            ROS_INFO("TRANSIT Error: Run LIFT command first.");
            last_response.response.data = "ERROR";
        }
        else if (droneState == LIFTING_OFF){
            ROS_INFO("TRANSIT Error: Wait for liftoff completion.");
            last_response.response.data = "ERROR";
        }
        else if (droneState == LANDING){
            ROS_INFO("TRANSIT Error: Wait for landing completion.");
            last_response.response.data = "ERROR";
        }
        else if (inputCmd == "TRANSIT_ADD"){
            Eigen::Vector3d starting;
            /*if there are no trajectories being followed, the starting point will start at the curr_position
              if there are trajectories being followered, the starting point will start the end of the last trajectory */
            if (droneState == HOVERING){
                starting = Eigen::Vector3d(hover_position.pose.position.x, hover_position.pose.position.y, hover_position.pose.position.z);
            }
            else{
                TrajectoryGroupedInfo *last_traj = curr_trajectories.back();
                starting = Eigen::Vector3d(last_traj->endPoint.pose.position.x, last_traj->endPoint.pose.position.y, last_traj->endPoint.pose.position.z);
            }
            Eigen::Vector3d ending(inputMsg->target.x, inputMsg->target.y, inputMsg->target.z);
            mav_trajectory_generation::Segment outputSegment(generateTraj(starting[0], starting[1], starting[2], ending[0], ending[1], ending[2]));

            geometry_msgs::PoseStamped startingPoint;
            startingPoint.pose.position.x = starting[0];
            startingPoint.pose.position.y = starting[1];
            startingPoint.pose.position.z = starting[2];
            geometry_msgs::PoseStamped endingPoint;
            endingPoint.pose.position.x = ending[0];
            endingPoint.pose.position.y = ending[1];
            endingPoint.pose.position.z = ending[2];

            TrajectoryGroupedInfo *outputGroupedInfo = new TrajectoryGroupedInfo(outputSegment, startingPoint, endingPoint, static_cast<float>(outputSegment.getTime()));
            curr_trajectories.push(outputGroupedInfo);

            droneState = IN_TRANSIT;
            last_response.response.data = "RECIEVED";
        }
        else{
            hover_position.pose.position.x = curr_position.pose.position.x;
            hover_position.pose.position.y = curr_position.pose.position.y;
            hover_position.pose.position.z = curr_position.pose.position.z;
            ros::Rate temprate(20.0);
            // hover and stablize, for a little bit, before a new trajectory.
            ROS_INFO("STABLIZING...");
            while (!isFlat(curr_position.pose, .02) && !isStationary(curr_velocity, .01, .01)){
                local_pos_pub.publish(hover_position);
                ros::spinOnce();
                temprate.sleep();
            }
            ROS_INFO("STABLIZED!");
            while (curr_trajectories.size() != 0){
                TrajectoryGroupedInfo *first_traj = curr_trajectories.front();
                delete first_traj;
                curr_trajectories.pop();
            }
            currTrajTime = 0;
            currTime = 0;

            Eigen::Vector3d starting(curr_position.pose.position.x, curr_position.pose.position.y, curr_position.pose.position.z);
            Eigen::Vector3d ending(inputMsg->target.x, inputMsg->target.y, inputMsg->target.z);
            mav_trajectory_generation::Segment outputSegment(generateTraj(starting[0], starting[1], starting[2], ending[0], ending[1], ending[2]));

            geometry_msgs::PoseStamped startingPoint;
            startingPoint.pose.position.x = starting[0];
            startingPoint.pose.position.y = starting[1];
            startingPoint.pose.position.z = starting[2];
            geometry_msgs::PoseStamped endingPoint;
            endingPoint.pose.position.x = ending[0];
            endingPoint.pose.position.y = ending[1];
            endingPoint.pose.position.z = ending[2];

            TrajectoryGroupedInfo *outputGroupedInfo = new TrajectoryGroupedInfo(outputSegment, startingPoint, endingPoint, static_cast<float>(outputSegment.getTime()));
            curr_trajectories.push(outputGroupedInfo);

            droneState = IN_TRANSIT;
            last_response.response.data = "RECIEVED";
        }
    }
    else{
        ROS_INFO("Error: Invalid Command.");
        last_response.response.data = "ERROR";
    }
    multi_info_pub.publish(last_response);
}
