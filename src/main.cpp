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
#include <ros/console.h>
#include <vector>
#include <iostream>

typedef struct rpy{
    
    double roll;
    double pitch;
    double yaw;

}rpy;

typedef struct dtVelocity{

    double vx;
    double vy;
    double vz;

}dtVelocity;

rpy RPY;
dtVelocity dtVel;

void callbackTrueBody(const vectornav::trueBody::ConstPtr& data){
                
    RPY.roll      = data->RPY.x;
    RPY.pitch     = data->RPY.y;
    RPY.yaw       = data->RPY.z;

    // acc[0]      = data->bodyAcc.x;
    // acc[1]      = data->bodyAcc.y;
    // acc[2]      = data->bodyAcc.z;

    // angular[0]  = data->gyro.x;
    // angular[1]  = data->gyro.y;
    // angular[2]  = data->gyro.z;

}

void callbackDThetaVel(const vectornav::dThetaVel::ConstPtr& data){
       
    // dTheta[0]   = data->deltaTheta.x;
    // dTheta[1]   = data->deltaTheta.y;
    // dTheta[2]   = data->deltaTheta.z;

    dtVel.vx     = data->deltaVelocity.x;
    dtVel.vy     = data->deltaVelocity.y;
    dtVel.vz     = data->deltaVelocity.z;
    
}


class PositionEstimation
{
    private:
        double header;
        std::vector<double> dVel;
        std::vector<double> position;
        std::vector<double> lastPosition;
        std::vector<double> lastVelocity;
    public:
        // void PositionEstimation(void);
        void estimation(double dt, dtVelocity vel){
            
            dVel[0]     = vel.vx;
            dVel[1]     = vel.vy;
            dVel[2]     = vel.vz;

            // std::cout << dVel << std::endl;

            // dVel.vx     = lastVel.vx + vel.vx;
            // dVel.vy     = lastVel.vy + vel.vy;
            // dVel.vz     = lastVel.vz + vel.vz;            

            // pos.px      = lastPos.px + dVel.vx * dt + 0.5 * dVel.vx;
            // pos.py      = lastPos.py + dVel.vy * dt + 0.5 * dVel.vy;
            // pos.pz      = lastPos.pz + dVel.vz * dt + 0.5 * dVel.vz;

            // lastVel.vx  = dVel.vx;
            // lastVel.vy  = dVel.vy;
            // lastVel.vz  = dVel.vz;

            // lastPos.px  = pos.px;
            // lastPos.py  = pos.py;
            // lastPos.pz  = pos.pz; 
            
        }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "OrnibibotPC");
    
    ros::NodeHandle nh;

    PositionEstimation pse  = PositionEstimation();

    ros::Subscriber sub_trueBody    = nh.subscribe("vectornav/trueBody", 1000, &callbackTrueBody);
    ros::Subscriber sub_dThetaVel   = nh.subscribe("vectornav/dThetaVel", 1000, &callbackDThetaVel);
    ros::Publisher odom_pub         = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Time time, lastTime;

    ros::Rate rate(100);
    
    while(ros::ok){

        time        = ros::Time::now();
        double dt   = (time-lastTime).toSec();

        pse.estimation(dt, dtVel);

        lastTime    = time;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}