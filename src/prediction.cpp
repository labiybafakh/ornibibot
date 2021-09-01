#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


#define ts 0.005
#define h 4

using namespace ros;

float integratedX[5],integratedY[5],integratedZ[5];
float intX,intY, intZ;
float position[3];
int iteration=0;
double ax, ay, az;
double roll,pitch,yaw;
float vx,vy,vz,sx,sy,sz;
double current_time,last_time;

void getPosition(int ax, int ay, int az);

void accCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ax    = msg->data[0];
  ay    = msg->data[1];
  az    = msg->data[2];
  roll  = msg->data[3];
  pitch = msg->data[4];
  yaw   = msg->data[5];

//   if(yaw >= 0 && yaw <=180)
//     yaw = yaw / 57.2958;
//   else
//     yaw = ((360-yaw) * -1) / 57.2958;
  yaw = yaw / 57.2958;
   

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

void getPosition(int ax, int ay, int az){
    //Function of simpson rule integration

    vx      = vx + ((lastAx + ax) * 0.5) 
   
    intX    = calcDisplacement(ax);
    intY    = calcDisplacement(ay);
    intZ    = calcDisplacement(az);

    //Calculate final distance from integration        
    position[0] += intX;     
    position[1] += intY;
    position[2] += intZ;



}

int main(int argc, char **argv){

    init(argc, argv, "OrnibiBotPC");

    NodeHandle n;
    // tf::TransfromBroadcaster odom_broadcaster;

    Subscriber sub = n.subscribe("Nav", 1, accCallback);
    // Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
    
    // current_time = Time::now();
    // last_time = Time::now();

    Rate loop_rate(200);

    while(ok){
        
        current_time = Time::now().toSec();
        
        getPosition(ax, ay, az);        
        
        // printf("\n\r%f\t%f\t%f\t%f\t%f\t%f\t%f",position[0],position[1],position[2], ax, ay, az, yaw);
        printf("\n\r%f\t%f",current_time, last_time);

        last_time = current_time;

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = position[0];
        transformStamped.transform.translation.y = position[1];
        transformStamped.transform.translation.z = position[2];

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
        
        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}