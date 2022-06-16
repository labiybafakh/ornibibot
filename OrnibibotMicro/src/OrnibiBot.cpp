#include "OrnibibBot.h"

OrnibiBot::OrnibiBot(ros::NodeHandle * nodehandle):nh_(*nodehandle){
    initPublisher();
    initSubscriber();
}

void OrnibiBot::initPublisher(){
    // nh_.advertise(* chatter("chatter", &str_msg));
    std_msgs::Int16 str_msg;
    std_msgs::Int16 cnt_msg;

    chatterPub = chatter("chatter", &str_msg);
    cntPub = cnt("cntr", &cnt_msg);

    nh_.advertise();
    nh_.advertise(("cntr", &cnt_msg);
}

void OrnibiBot::initSubscriber(){
    nh_.subscribe(* subFlap);
}

void OrnibiBot::freq_cb(const geometry_msgs::Twist& flap){
    freqFlap = flap.linear.x;
}
























































void OrnibiBot::initSubscriber(){
    nh_.subscribe(* subFlap);
}

void OrnibiBot::freq_cb(const geometry_msgs::Twist& msg){
    freqFlap = msg.linear.x;
}