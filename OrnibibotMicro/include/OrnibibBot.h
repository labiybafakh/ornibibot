#ifndef ORNIBIBOT_H
#define ORNIBIBOT_H

#include <Arduino.h>
#include <WiFi.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>
#include <OrnibibBot.h>

class OrnibiBot{
    public:
        OrnibiBot(ros::NodeHandle* nodehandle);
        void freq_cb(const geometry_msgs::Twist& msg);
        volatile double freqFlap;

    protected:
        ros::NodeHandle nh_;
        std_msgs::Int16 str_msg;
        std_msgs::Int16 cnt_msg;
        const uint16_t serverPort = 11411;
        void initSubscriber();
        void initPublisher();
        ros::Publisher chatterPub;
        ros::Publisher cntPub;
        // ros::Subscriber<geometry_msgs::Twist> flap("flapFreq", freq_cb);
        ros::Subscriber<geometry_msgs::Twist, OrnibiBot> * subFlap;
        // ros::Publisher * chatter;
        // ros::Publisher * cnt;

};

#endif