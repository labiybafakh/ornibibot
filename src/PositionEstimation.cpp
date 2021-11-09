#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <vectornav/trueBody.h>
#include <vectornav/dThetaVel.h>

class PositionEstimation
{
private:
    ros::Publisher  
public:
    posEst(ros::NodeHandle *nh)
    {

    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "OrnibibotPC");
    ros::NodeHandle nh;

    PositionEstimation positionEstimation(&nh);
    
}