#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <vectornav/trueBody.h>
#include <vectornav/dThetaVel.h>
#include <ros/console.h>
#include <vector>
#include <iostream>

// #define M_PI  3.14159265358979323846f  /* pi */
volatile double freqFlap=1;

volatile uint16_t counter=0;

volatile double getFlapMs(double freq){
  return (volatile double)(1000/freq);
}

volatile int16_t interpolateFlap(int amplitude, volatile double freq, int time){
    
    return (volatile int16_t) (amplitude * sin(((2*M_PI)/getFlapMs(freqFlap) * time)));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OrnibibotPC");
    
    ros::NodeHandle nh;

    ros::Publisher pub         = nh.advertise<std_msgs::Int16>("signal", 100);

    ros::Time time, lastTime;

    ros::Rate rate(1000);
    
    while(ros::ok){

        time        = ros::Time::now();
        double dt   = (time-lastTime).toSec();

        ROS_INFO("%d\t%d\t%f", (int) (400 * sin(((2*M_PI)/getFlapMs(freqFlap)) * counter)), interpolateFlap(400, getFlapMs(freqFlap), counter), getFlapMs(freqFlap));
        if(counter<getFlapMs(freqFlap))counter++;
        else counter=0;  

        lastTime    = time;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}