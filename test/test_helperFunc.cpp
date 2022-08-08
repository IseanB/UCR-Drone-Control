#include <gtest/gtest.h>
#include "../helper/computations.h"

TEST(isStationary, allPosAxis){
    geometry_msgs::Twist xaxis_linear_vel;
    geometry_msgs::Twist yaxis_linear_vel;
    geometry_msgs::Twist zaxis_linear_vel;

    geometry_msgs::Twist xaxis_angular_vel;
    geometry_msgs::Twist yaxis_angular_vel;
    geometry_msgs::Twist zaxis_angular_vel;

    xaxis_linear_vel.linear.x = 2;
    yaxis_linear_vel.linear.y = 2;
    zaxis_linear_vel.linear.z = 2;

    xaxis_angular_vel.angular.x = 2;
    yaxis_angular_vel.angular.y = 2;
    zaxis_angular_vel.angular.z = 2;

    EXPECT_EQ(isStationary(xaxis_linear_vel,1.99), false);
    EXPECT_EQ(isStationary(yaxis_linear_vel,1.99), false);
    EXPECT_EQ(isStationary(zaxis_linear_vel,1.99), false);

    EXPECT_EQ(isStationary(xaxis_linear_vel,2), true);
    EXPECT_EQ(isStationary(yaxis_linear_vel,2), true);
    EXPECT_EQ(isStationary(zaxis_linear_vel,2), true);

    EXPECT_EQ(isStationary(xaxis_linear_vel,2.1), true);
    EXPECT_EQ(isStationary(yaxis_linear_vel,2.1), true);
    EXPECT_EQ(isStationary(zaxis_linear_vel,2.1), true);


    EXPECT_EQ(isStationary(xaxis_angular_vel,0,1.99), false);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,1.99), false);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,1.99), false);

    EXPECT_EQ(isStationary(xaxis_angular_vel,0,2), true);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,2), true);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,2), true);

    EXPECT_EQ(isStationary(xaxis_angular_vel,0,2.99), true);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,2.99), true);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,2.99), true);
    
}

TEST(isStationary, allNegAxis){
    geometry_msgs::Twist xaxis_vel;
    geometry_msgs::Twist yaxis_vel;
    geometry_msgs::Twist zaxis_vel;

    geometry_msgs::Twist xaxis_angular_vel;
    geometry_msgs::Twist yaxis_angular_vel;
    geometry_msgs::Twist zaxis_angular_vel;

    xaxis_vel.linear.x = -2;
    yaxis_vel.linear.y = -2;
    zaxis_vel.linear.z = -2;

    xaxis_angular_vel.angular.x = 2;
    yaxis_angular_vel.angular.y = 2;
    zaxis_angular_vel.angular.z = 2;

    EXPECT_EQ(isStationary(xaxis_vel,1.99), false);
    EXPECT_EQ(isStationary(yaxis_vel,1.99), false);
    EXPECT_EQ(isStationary(zaxis_vel,1.99), false);

    EXPECT_EQ(isStationary(xaxis_vel,2), true);
    EXPECT_EQ(isStationary(yaxis_vel,2), true);
    EXPECT_EQ(isStationary(zaxis_vel,2), true);

    EXPECT_EQ(isStationary(xaxis_vel,2.1), true);
    EXPECT_EQ(isStationary(yaxis_vel,2.1), true);
    EXPECT_EQ(isStationary(zaxis_vel,2.1), true);
    
    
    EXPECT_EQ(isStationary(xaxis_angular_vel,0,1.99), false);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,1.99), false);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,1.99), false);

    EXPECT_EQ(isStationary(xaxis_angular_vel,0,2), true);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,2), true);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,2), true);

    EXPECT_EQ(isStationary(xaxis_angular_vel,0,2.99), true);
    EXPECT_EQ(isStationary(yaxis_angular_vel,0,2.99), true);
    EXPECT_EQ(isStationary(zaxis_angular_vel,0,2.99), true);
}

TEST(isStationary, posVel){
    geometry_msgs::Twist drone1;
    geometry_msgs::Twist drone2;
    geometry_msgs::Twist drone3;

    drone1.linear.x = 3;
    drone1.angular.y = 5;
    drone1.linear.z = 4;

    drone2.angular.x = 4.5;
    drone2.linear.y = 1.1;
    drone2.linear.z = 4.4;

    drone3.angular.x = .1;
    drone3.linear.y = .15;
    drone3.angular.z = .2;

    EXPECT_EQ(isStationary(drone1,4.9,4.9), false);
    EXPECT_EQ(isStationary(drone1,5,5), true);
    EXPECT_EQ(isStationary(drone1,5.1,5.1), true);

    EXPECT_EQ(isStationary(drone2,4.5,4.4), false);
    EXPECT_EQ(isStationary(drone2,4.6,4.5), true);
    EXPECT_EQ(isStationary(drone2,5,4.6), true);

    EXPECT_EQ(isStationary(drone3,.1,.14), false);
    EXPECT_EQ(isStationary(drone3,.2,.2), false);
    EXPECT_EQ(isStationary(drone3,.3,.25), true);
}

TEST(isStationary, negVel){
    geometry_msgs::Twist drone1;
    geometry_msgs::Twist drone2;
    geometry_msgs::Twist drone3;

    drone1.linear.x = -3;
    drone1.linear.y = -4;
    drone1.linear.z = -0;

    drone2.linear.x = -1.1;
    drone2.linear.y = -2.3;
    drone2.linear.z = -4.4;

    drone3.linear.x = -.1;
    drone3.linear.y = -0;
    drone3.linear.z = -.2;

    EXPECT_EQ(isStationary(drone1,4.9), false);
    EXPECT_EQ(isStationary(drone1,5), true);
    EXPECT_EQ(isStationary(drone1,5.1), true);

    EXPECT_EQ(isStationary(drone2,5), false);
    EXPECT_EQ(isStationary(drone2,5.1), true);
    EXPECT_EQ(isStationary(drone2,6), true);

    EXPECT_EQ(isStationary(drone3,.1), false);
    EXPECT_EQ(isStationary(drone3,.2), false);
    EXPECT_EQ(isStationary(drone3,.3), true);
}

TEST(isStationary, posAndNegativeVel){
    geometry_msgs::Twist drone1;
    geometry_msgs::Twist drone2;
    geometry_msgs::Twist drone3;

    drone1.linear.x = -3;
    drone1.linear.y = 4;
    drone1.linear.z = 0;

    drone2.linear.x = 1.1;
    drone2.linear.y = -2.3;
    drone2.linear.z = -4.4;

    drone3.linear.x = .1;
    drone3.linear.y = 0;
    drone3.linear.z = -.2;

    EXPECT_EQ(isStationary(drone1,4.9), false);
    EXPECT_EQ(isStationary(drone1,5), true);
    EXPECT_EQ(isStationary(drone1,5.1), true);

    EXPECT_EQ(isStationary(drone2,5), false);
    EXPECT_EQ(isStationary(drone2,5.1), true);
    EXPECT_EQ(isStationary(drone2,6), true);

    EXPECT_EQ(isStationary(drone3,.1), false);
    EXPECT_EQ(isStationary(drone3,.2), false);
    EXPECT_EQ(isStationary(drone3,.3), true);
}

TEST(isFlat, posTilt){
    geometry_msgs::Pose drone1;
    geometry_msgs::Pose drone2;
    geometry_msgs::Pose drone3;

    drone1.orientation.x = .1;
    drone1.orientation.y = 0;
    drone2.orientation.x = 2;
    drone2.orientation.y = 2.3;
    drone3.orientation.x = 5;
    drone3.orientation.y = 5;

    EXPECT_EQ(isFlat(drone1,.1), true);
    EXPECT_EQ(isFlat(drone1,-.09), false);

    EXPECT_EQ(isFlat(drone2,2.4), true);
    EXPECT_EQ(isFlat(drone2,2), false);

    EXPECT_EQ(isFlat(drone3,5.1), true);
    EXPECT_EQ(isFlat(drone3,-5), true);
}

TEST(isFlat, negTilt){
    geometry_msgs::Pose drone1;
    geometry_msgs::Pose drone2;
    geometry_msgs::Pose drone3;

    drone1.orientation.x = -.1;
    drone1.orientation.y = 0;
    drone2.orientation.x = -2;
    drone2.orientation.y = -2.3;
    drone3.orientation.x = -5;
    drone3.orientation.y = -5;

    EXPECT_EQ(isFlat(drone1,.1), true);
    EXPECT_EQ(isFlat(drone1,-.09), false);

    EXPECT_EQ(isFlat(drone2,2.4), true);
    EXPECT_EQ(isFlat(drone2,2), false);

    EXPECT_EQ(isFlat(drone3,5.1), true);
    EXPECT_EQ(isFlat(drone3,-5), true);
}

TEST(reachedLocation_Pose, posValues){
    geometry_msgs::Pose drone1;
        drone1.position.x = 0;
        drone1.position.y = 0;
        drone1.position.z = 0;
    geometry_msgs::Pose drone2;
        drone2.position.x = 1;
        drone2.position.y = 0;
        drone2.position.z = 0;
    geometry_msgs::Pose drone3;
        drone3.position.x = 0;
        drone3.position.y = 1;
        drone3.position.z = 0;
    geometry_msgs::Pose drone4;
        drone4.position.x = 0;
        drone4.position.y = 0;
        drone4.position.z = 1;
    geometry_msgs::Pose drone5;
        drone5.position.x = 1;
        drone5.position.y = 1;
        drone5.position.z = 0;
    geometry_msgs::Pose drone6;
        drone6.position.x = 0;
        drone6.position.y = 1;
        drone6.position.z = 1;
    geometry_msgs::Pose drone7;
        drone7.position.x = 1;
        drone7.position.y = 0;
        drone7.position.z = 1;
    geometry_msgs::Pose drone8;
        drone8.position.x = 1;
        drone8.position.y = 1;
        drone8.position.z = 1;
    geometry_msgs::Pose drone9;
        drone9.position.x = .9;
        drone9.position.y = .4;
        drone9.position.z = 0;

    geometry_msgs::PoseStamped destination;
    destination.pose.position.x = 0;
    destination.pose.position.y = 0;
    destination.pose.position.z = 0;

    EXPECT_EQ(reachedLocation(drone1, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone2, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone3, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone4, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone5, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone6, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone7, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone8, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone9, destination, 2), true);

}

TEST(reachedLocation_Pose, negValues){
        geometry_msgs::Pose drone1;
        drone1.position.x = 0;
        drone1.position.y = 0;
        drone1.position.z = 0;
    geometry_msgs::Pose drone2;
        drone2.position.x = -1;
        drone2.position.y = 0;
        drone2.position.z = 0;
    geometry_msgs::Pose drone3;
        drone3.position.x = 0;
        drone3.position.y = -1;
        drone3.position.z = 0;
    geometry_msgs::Pose drone4;
        drone4.position.x = 0;
        drone4.position.y = 0;
        drone4.position.z = -1;
    geometry_msgs::Pose drone5;
        drone5.position.x = -1;
        drone5.position.y = 1;
        drone5.position.z = 0;
    geometry_msgs::Pose drone6;
        drone6.position.x = 0;
        drone6.position.y = -1;
        drone6.position.z = 1;
    geometry_msgs::Pose drone7;
        drone7.position.x = -1;
        drone7.position.y = 0;
        drone7.position.z = -1;
    geometry_msgs::Pose drone8;
        drone8.position.x = 1;
        drone8.position.y = -1;
        drone8.position.z = -1;
    geometry_msgs::Pose drone9;
        drone9.position.x = -.9;
        drone9.position.y = .4;
        drone9.position.z = 0;

    geometry_msgs::PoseStamped destination;
    destination.pose.position.x = 0;
    destination.pose.position.y = 0;
    destination.pose.position.z = 0;

    EXPECT_EQ(reachedLocation(drone1, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone2, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone3, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone4, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone5, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone6, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone7, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone8, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone9, destination, 2), true);

}

TEST(reachedLocation_PoseStamped, posValues){
        geometry_msgs::Pose drone1;
        drone1.position.x = 0;
        drone1.position.y = 0;
        drone1.position.z = 0;
    geometry_msgs::Pose drone2;
        drone2.position.x = 1;
        drone2.position.y = 0;
        drone2.position.z = 0;
    geometry_msgs::Pose drone3;
        drone3.position.x = 0;
        drone3.position.y = 1;
        drone3.position.z = 0;
    geometry_msgs::Pose drone4;
        drone4.position.x = 0;
        drone4.position.y = 0;
        drone4.position.z = 1;
    geometry_msgs::Pose drone5;
        drone5.position.x = 1;
        drone5.position.y = 1;
        drone5.position.z = 0;
    geometry_msgs::Pose drone6;
        drone6.position.x = 0;
        drone6.position.y = 1;
        drone6.position.z = 1;
    geometry_msgs::Pose drone7;
        drone7.position.x = 1;
        drone7.position.y = 0;
        drone7.position.z = 1;
    geometry_msgs::Pose drone8;
        drone8.position.x = 1;
        drone8.position.y = 1;
        drone8.position.z = 1;
    geometry_msgs::Pose drone9;
        drone9.position.x = .9;
        drone9.position.y = .4;
        drone9.position.z = 0;

    geometry_msgs::Point destination;
    destination.x = 0;
    destination.y = 0;
    destination.z = 0;

    EXPECT_EQ(reachedLocation(drone1, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone2, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone3, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone4, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone5, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone6, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone7, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone8, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone9, destination, 2), true);

}

TEST(reachedLocation_PoseStamped, negValues){
            geometry_msgs::Pose drone1;
        drone1.position.x = 0;
        drone1.position.y = 0;
        drone1.position.z = -0;
    geometry_msgs::Pose drone2;
        drone2.position.x = -1;
        drone2.position.y = 0;
        drone2.position.z = 0;
    geometry_msgs::Pose drone3;
        drone3.position.x = 0;
        drone3.position.y = -1;
        drone3.position.z = 0;
    geometry_msgs::Pose drone4;
        drone4.position.x = 0;
        drone4.position.y = 0;
        drone4.position.z = -1;
    geometry_msgs::Pose drone5;
        drone5.position.x = -1;
        drone5.position.y = 1;
        drone5.position.z = 0;
    geometry_msgs::Pose drone6;
        drone6.position.x = 0;
        drone6.position.y = -1;
        drone6.position.z = 1;
    geometry_msgs::Pose drone7;
        drone7.position.x = 1;
        drone7.position.y = 0;
        drone7.position.z = -1;
    geometry_msgs::Pose drone8;
        drone8.position.x = -1;
        drone8.position.y = -1;
        drone8.position.z = -1;
    geometry_msgs::Pose drone9;
        drone9.position.x = -.9;
        drone9.position.y = -.4;
        drone9.position.z = 0;

    geometry_msgs::Point destination;
    destination.x = 0;
    destination.y = 0;
    destination.z = 0;

    EXPECT_EQ(reachedLocation(drone1, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone2, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone3, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone4, destination, 1), true);
    EXPECT_EQ(reachedLocation(drone5, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone6, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone7, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone8, destination, 1), false);
    EXPECT_EQ(reachedLocation(drone9, destination, 2), true);

}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

/*
Makes sures this_pos is not tilted more than maxTilt, degrees, in any axis
bool isFlat(const geometry_msgs::Pose this_pos, float maxTilt);

Finds magnitude of this_vel values, x and y and z, checks its less than or equal to maxSpeed(m/s)
bool isStationary(const geometry_msgs::Twist this_vel, float maxSpeed);

Finds distance from curr_pos to desired_pos and checks it's within a accuracyDistance
bool reachedLocation(const geometry_msgs::Pose this_pos, const geometry_msgs::PoseStamped desired_pos, float accuracyDistance);
*/