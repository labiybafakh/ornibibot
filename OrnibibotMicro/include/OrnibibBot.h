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
    private:
        volatile uint16_t getFlapMs(volatile double freq);

    public:
        volatile uint16_t _time;
        volatile uint16_t _amplitude;
        volatile double _flapFreq;
        volatile uint16_t _periode;
        volatile uint16_t flapping;

        volatile int16_t sineFlap();
        volatile int16_t squareFlap();
        volatile int16_t sawFlap();

};

#endif