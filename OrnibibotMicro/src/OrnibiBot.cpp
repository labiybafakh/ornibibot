#include "OrnibibBot.h"

// IPAddress server(192,168,30,243);

OrnibiBot::OrnibiBot(ros::NodeHandle * nodehandle):nh_(*nodehandle), chatterPub("chatter", &str_msg), cntPub("cntr", &cnt_msg){
    // this ->nh_.getHardware(server, serverPort);
    this -> nh_.initNode();
    initPublisher();
    initSubscriber();
}

void OrnibiBot::initPublisher(){

    nh_.advertise(chatterPub);
    nh_.advertise(cntPub);

}

void OrnibiBot::initSubscriber(){
    nh_.subscribe(* subFlap);
}

void OrnibiBot::freq_cb(const geometry_msgs::Twist& flap){
    freqFlap = flap.linear.x;
}
