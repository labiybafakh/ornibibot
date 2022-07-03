#include "SBUS.h"
#include <Arduino.h>



// SBUS::SBUS(){

// }

void SBUS::init(){
    Serial2.begin(sbus_speed, SERIAL_8E2, 16, 17, true); 
}

int SBUS::degToSignal(int pos){
    //Min pulse 920, Max pulse 2120, Mid Pulse 1520
    return (int)(1220 + pos*3);
}
void SBUS::setPosition(int pos[]){

    sbus_servo_id[0] = SBUS::degToSignal(pos[0]);
    sbus_servo_id[1] = SBUS::degToSignal(pos[1]);
    sbus_servo_id[2] = SBUS::degToSignal(pos[2]);
    sbus_servo_id[3] = SBUS::degToSignal(pos[3]);
    sbus_servo_id[4] = 0;
    sbus_servo_id[5] = 0;
    
    sbus_data[0] = 0x0f;
    sbus_data[1] =  sbus_servo_id[0] & 0xff;
    sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
    sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
    sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
    sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
    sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
    sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
    sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
    sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

}

bool SBUS::sendPosition(){
    return Serial2.write(sbus_data, 25);
}