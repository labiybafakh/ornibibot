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

#define ts 0.005
#define h 4

using namespace ros;

float integratedX[5],integratedY[5],integratedZ[5];
float intX,intY, intZ;
float position[3];
int iteration=0;
double ax, ay, az;
double roll,pitch,yaw; 
double dt;
double currentSx,lastSx,futureSx;
double currentSy,lastSy,futureSy;
double currentSz,lastSz,futureSz;
double posX,posY;

void getPosition(double ax, double ay, double az, double yaw);

void accCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ax    = msg->data[0];
  ay    = msg->data[1];
  az    = msg->data[2];
  roll  = msg->data[3];
  pitch = msg->data[4];
  yaw   = msg->data[5];

  if(yaw >= 0 && yaw <=180)
    yaw = yaw / 57.2958;
  else
    yaw = ((360-yaw) * -1) / 57.2958;
//   yaw = yaw / 57.2958;
   

}

float calcDisplacement(float acc){
    //Function to calculate distance from acceleration
    float output=0;

    // velocity = velocity + acc * ts;
    // distance = distance * ts;

    return output;
}

void simpsonRule(int ax, int ay, int az){
    //Function of simpson rule integration
    if(iteration<5){

        integratedX[iteration] = calcDisplacement(ax);
        integratedY[iteration] = calcDisplacement(ay);
        integratedZ[iteration] = calcDisplacement(az);

        iteration++;
    }       
    else{
    
        //Calculate final distance from integration        
        position[0] += (h/3) * (integratedX[0] + (4*integratedX[1]) + (2*integratedX[2]) + (4*integratedX[3]) + integratedX[4]);     
        position[1] += (h/3) * (integratedY[0] + (4*integratedY[1]) + (2*integratedY[2]) + (4*integratedY[3]) + integratedY[4]);
        position[2] += (h/3) * (integratedZ[0] + (4*integratedZ[1]) + (2*integratedZ[2]) + (4*integratedZ[3]) + integratedZ[4]);

        
        iteration = 0;
    }
}

void getPosition(double ax, double ay, double az, double yaw){
    //Function of simpson rule integration
    currentSx   = futureSx;
    currentSy   = futureSy;
    currentSz   = futureSz;

    posX        = (currentSx * cos(yaw)) - (currentSy * sin(yaw));
    posY        = (currentSx * sin(yaw)) + (currentSy * cos(yaw));

    futureSx    = currentSx + (currentSx-lastSx) + (ax * dt *dt);
    lastSx      = currentSx;
    futureSy    = currentSy + (currentSy-lastSy) + (ay * dt *dt);
    lastSy      = currentSy;
    futureSz    = currentSz + (currentSz-lastSz) + (az * dt *dt);
    lastSz      = currentSz;

    futureSx    = currentSx + (currentSx-lastSx) + (ax * dt *dt);
    lastSx      = currentSx;
    futureSy    = currentSy + (currentSy-lastSy) + (ay * dt *dt);
    lastSy      = currentSy;
    futureSz    = currentSz + futureSz + (az * dt *dt);
    lastSz      = currentSz;
}

int main(int argc, char **argv){

    init(argc, argv, "OrnibiBotPC");

    NodeHandle n;
    // tf::TransfromBroadcaster odom_broadcaster;

    Subscriber sub = n.subscribe("Nav", 1, accCallback);
    Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
    Time current_time,last_time;
    // current_time = Time::now();
    // last_time = Time::now();

    Rate loop_rate(200);

    while(ok){

        spinOnce();       

        current_time= Time::now();
        dt          = (current_time - last_time).toSec();

        getPosition(ax, ay, az, yaw); 
        // printf("\n\r%f\t%f\t%f\t%f\t%f\t%f\t%f",position[0],position[1],position[2], ax, ay, az, yaw);
        // printf("\n\r%f\t%f",current_time, last_time);
        last_time = current_time;

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = posX;
        transformStamped.transform.translation.y = posY;
        transformStamped.transform.translation.z = currentSz;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, yaw);

        transformStamped.transform.rotation.x = quat_tf.x();
        transformStamped.transform.rotation.y = quat_tf.y();
        transformStamped.transform.rotation.z = quat_tf.z();
        transformStamped.transform.rotation.w = quat_tf.w();

        br.sendTransform(transformStamped);

        ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t%f",currentSx,currentSy,currentSz, ax, ay, az, yaw);
        
        //ROS_INFO("%f",dt);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion quat_odom;

        tf2::convert(quat_odom , quat_tf);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = posX;
        odom.pose.pose.position.y = posY;
        odom.pose.pose.position.z = currentSz;
        odom.pose.pose.orientation = quat_odom;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        //publish the message
        odom_pub.publish(odom);
        
        loop_rate.sleep();
    }

    return 0;
}