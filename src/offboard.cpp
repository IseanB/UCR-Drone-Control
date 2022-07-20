#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>

mavros_msgs::State current_state;
geometry_msgs::Pose curr_position;
geometry_msgs::Twist curr_velocity;
bool state_liftOffDone = false;
bool state_inTransitDone = false;
bool state_landingDone = false;

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

bool liftOffProgressCheck(){
    if (!state_liftOffDone && abs(curr_velocity.linear.z) < .15 && abs(curr_position.position.z-2) < .05){
        ROS_INFO("Vehicle Has Lifted Off.");
        state_liftOffDone = true;
        return true;
    }
    return false;
}

bool inTransitProgressCheck(){
    if (!state_inTransitDone && abs(curr_velocity.linear.x) < .1 && (curr_position.orientation.y < .1) && abs(curr_position.position.x - 3) < .2 && state_liftOffDone){
        state_inTransitDone = true;
        ROS_INFO("Vehicle Has Reached It's Desitnation.");
        return true;
    }
    return false;
}

bool landingProgressCheck(){
    if (state_inTransitDone && state_liftOffDone && !state_landingDone && abs(curr_velocity.linear.z) > .001 && abs(curr_position.position.z - .15) < .01){
        state_inTransitDone = true;
        ROS_INFO("Vehicle Has Landed.");
        return true;
    }
    return false;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    setup();
    ros::NodeHandle nh;

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
            ("mavros/setpoint_position/local", 0, updatePose);
    ros::Publisher local_vel_updatevel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 0, updateVel);
    ros::Publisher local_vel_updatepose_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 0, updatePose);
    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::PoseStamped finalpose;
    finalpose.pose.position.x = 3;
    finalpose.pose.position.y = 0;
    finalpose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    //liftoff code
    while(ros::ok() && (!state_liftOffDone) && (!liftOffProgressCheck())){
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

        local_pos_pub.publish(pose);
        
        ros::spinOnce();
        rate.sleep();
    }

    //in transit code
    while(ros::ok() && state_liftOffDone && !inTransitProgressCheck()){
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            ROS_INFO("Vehicle In Motion");
            last_request = ros::Time::now();
        }
        local_pos_pub.publish(finalpose);
        ros::spinOnce();
        rate.sleep();
    }

    //landing code
    while(ros::ok() && state_liftOffDone && state_inTransitDone && !state_landingDone){
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            ROS_INFO("Vehicle Is Descending");
            last_request = ros::Time::now();
        }
        geometry_msgs::Twist updatingVel;
        if(curr_position.position.z > .1){
            updatingVel.linear.z = (-.7) * abs(curr_position.position.z-.8);
            local_vel_updatepose_pub.publish(updatingVel);
        } 
        if(curr_position.position.z < .2 || (curr_velocity.linear.z > .1 && curr_position.position.z < .4)){
            mavros_msgs::CommandLong endflight_call;
            endflight_call.request.broadcast = false;
            endflight_call.request.command = 400;
            endflight_call.request.param2 = 21196.0;
            if( end_flight_client.call(endflight_call) &&
                end_flight_client.waitForExistence()){
                ROS_INFO("Vehicle is Shutting Down");
            }
            state_landingDone = true;
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}

