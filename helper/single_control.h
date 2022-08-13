#ifndef SINGLE_CONTROL_H
#define SINGLE_CONTROL_H

/*  Custom files  */
#include "conversions.h"
#include "computations.h"

/*  Libraries   */
#include <ros/ros.h>

/*  Data Types  */
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>

enum PossiableState{
    IDLE,
    LIFTING_OFF,
    IN_TRANSIT,
    WAITING,
    LANDING,
    LANDED
};

enum PointType{
    RELATIVE,
    GLOBAL
};

class SingleDroneControl{
    public:
        SingleDroneControl(float max_velocity, float max_acceleration, int droneID = -1); 

        /*Long Control*/ 
        // void addTrajectory(/*vector of points*/);/**/
        // void skipCurrTrajectory(); //halts trajectory and goes immediate to start of next trajectory
        // void clearTrajectories(); 
        // void nextTraj(); //Executes next trajectory, if not near starting point of next traj it will go to that start.


        /*Quick Control*/
        // void goToPoint(/*One Point, RELATIVE OR NOT*/);
        // void goToPoints(/*Vector of Points, RELATIVE OR NOT*/);
        // void haltTraj(); //Halts current trajectory immediately, and doesn't go to the next trajectory.

        /*Takes control and executes a command*/
        bool liftOff();
        bool landNow();
        void shutOff();


        //Modifiers
        void changeMaxVelocity(float input_acc);
        void changeMaxAcceleration(float input_acc);

        //Accessors
        geometry_msgs::Twist currPosition() const;
        geometry_msgs::Pose currVelocity() const;
        int numberOfTrajs() const;

    private:
        float default_max_velocity;
        float default_max_acceleration;
        mav_trajectory_generation::Trajectory* current_trajectory;
        // queue<mav_trajectory_generation::Trajectory*> trajectory_queue;
       //No nodes, just adversite a single message

        geometry_msgs::Pose* curr_position;
        geometry_msgs::Twist* curr_velocity;
        PossiableState curr_state;

        mav_trajectory_generation::Trajectory* calculateTrajectory(float max_vel = -1, float max_accel = -1);
};

#endif