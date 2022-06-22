#include "OrnibibBot.h"


volatile uint16_t OrnibiBot::getFlapMs(volatile double freq){
  return (volatile uint16_t)(1000/freq);
}

volatile int16_t OrnibiBot::sineFlap(){
    _periode = OrnibiBot::getFlapMs(_flapFreq);

    return (volatile int16_t) (_amplitude * sin(((2*M_PI)/_periode * _time)));
}

volatile int16_t OrnibiBot::squareFlap(){
    return _time;
}

volatile int16_t OrnibiBot::sawFlap(){
    return _time;
} 
