#ifndef ORNIBIBOT_H
#define ORNIBIBOT_H

#include <Arduino.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>

class OrnibiBot{
    public:
        OrnibiBot(ros::NodeHandle* nodehandle);
        void freq_cb(const geometry_msgs::Twist& msg);
        volatile double freqFlap;

    private:
        ros::NodeHandle nh_;
        
        void initPublisher();
        void initSubscriber();
        ros::Publisher * chatterPub;
        ros::Publisher * cntPub;
        // ros::Subscriber<geometry_msgs::Twist> flap("flapFreq", freq_cb);
        ros::Subscriber<geometry_msgs::Twist, OrnibiBot> * subFlap;
        // ros::Publisher * chatter;
        // ros::Publisher * cnt;

};

#endif