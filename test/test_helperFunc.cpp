#include <gtest/gtest.h>
#include "../helper/helperFunc.h"

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