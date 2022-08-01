#include <ros/ros.h>
#include "../helper/helperFunc.h"

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include "mav_trajectory_generation/segment.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>

mavros_msgs::State current_state;
geometry_msgs::Pose curr_position;
geometry_msgs::Twist curr_velocity;

enum PossiableState{
    LIFTING_OFF,
    IN_TRANSIT,
    LANDING,
    LANDED
};

/* Assumes drone is on ground. Initializes curr_position & curr_velocity values to 0 */
void setup();

/* Used for initiating contact with drone and setting it to OFFBOARD mode */
void state_cb(const mavros_msgs::State::ConstPtr& msg);

/* Updates curr_position values, position and orientation, with inputted pose */
void updatePose(const geometry_msgs::PoseStamped::ConstPtr& inputPose);

/* Updates curr_velocity values, linear and angular, with inputted pose */
void updateVel(const geometry_msgs::TwistStamped::ConstPtr& inputPose);

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    setup();
    ros::NodeHandle nh;
    PossiableState droneState = LIFTING_OFF;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 0, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 0, updatePose);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 0, updateVel);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient end_flight_client = nh.serviceClient<mavros_msgs::CommandLong>
            ("mavros/cmd/command");        

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 0);
    ros::Publisher local_vel_update = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 0);

    
    //Code from github
    ////////////////////////////////////////////
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
    vertices.push_back(start);

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
    vertices.push_back(middle);

    end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
    vertices.push_back(end);
    
    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    const int N = 10;// needs to be at least 10 for snap, 8 for jerk
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    mav_trajectory_generation::Segment::Vector segments; //stores all points needed to follow
    opt.getSegments(&segments);// fills "segments" variable with segments of the path

    ////////////////////////////////////////////


    mav_trajectory_generation::Trajectory trajectory;
    trajectory.setSegments(segments);// dont use addSegments, uncessary calculations

    // std::cout << "Trajectory Properties:\n" << std::endl;
    // std::cout << "Number of Dimensions :  "<<trajectory.D() << std::endl;
    // std::cout << "Polynomial Order of Optimizination Function :  "<<trajectory.N() << std::endl;
    // std::cout << "Number of Segments :  " << trajectory.K() << std::endl << std::endl;
    // std::cout << "Optimization Function Information: \n\n"; 
    // printSegment(std::cout, segments.at(0), 0);
    // printSegment(std::cout, segments.at(1), 0);

    geometry_msgs::PoseStamped liftOffPos;
    liftOffPos.pose.position.x = 0;
    liftOffPos.pose.position.y = 0;
    liftOffPos.pose.position.z = 2;

    geometry_msgs::PoseStamped preLandPos;
    preLandPos.pose.position.x = 3;
    preLandPos.pose.position.y = 0;
    preLandPos.pose.position.z = 2;

    geometry_msgs::PoseStamped landingPos;
    landingPos.pose.position.x = 3;
    landingPos.pose.position.y = 0;
    landingPos.pose.position.z = 0;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(liftOffPos);
        ros::spinOnce();
        rate.sleep();
    }

    /*Used to send periodic messages about drone*/
    ros::Time last_request = ros::Time::now();

    //Liftoff Setup

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //Liftoff code

    while(ros::ok() && (droneState == LIFTING_OFF)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(liftOffPos);
        
        ros::spinOnce();
        rate.sleep();
        
        /*Checks if drone landed*/
        if((reachedLocation(curr_position, liftOffPos, .1)) && (isStationary(curr_velocity, .05))){
            droneState = IN_TRANSIT;
        }
    }


    //In Transit code 

    while(ros::ok() && (droneState == IN_TRANSIT)){
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            ROS_INFO("Vehicle In Motion");
            last_request = ros::Time::now();
        }

        local_pos_pub.publish(preLandPos);
        
        ros::spinOnce();
        rate.sleep();

        /*Checks if drone is done traveling*/
        if(reachedLocation(curr_position, preLandPos, .1) && isStationary(curr_velocity, .5) && isFlat(curr_position, 3.14/12)){
            droneState = LANDING;
        }
    }


    //Landing code

    while(ros::ok() && (droneState == LANDING)){
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            ROS_INFO("Vehicle Is Descending");
            last_request = ros::Time::now();
        }
        geometry_msgs::Twist updatingVel;
        if(curr_position.position.z > .1){
            updatingVel.linear.z = (-.2) * abs(curr_position.position.z - .1);
            local_vel_update.publish(updatingVel);
        } 
        if(isStationary(curr_velocity, .5) || ( curr_position.position.z < .1)){
            mavros_msgs::CommandLong endflight_call;
            endflight_call.request.broadcast = false;
            endflight_call.request.command = 400;
            endflight_call.request.param2 = 21196.0;
            if( end_flight_client.call(endflight_call) &&
                end_flight_client.waitForExistence()){
                ROS_INFO("Vehicle is Shutting Down");
            }
            droneState = LANDED;
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();

    }

    return 1;
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
