#ifndef SBUS_H
#define SBUS_H

class SBUS{
    protected:
        int sbus_speed = 100000;    
        int sbus_servo_id[16];
        char sbus_data[25] = {
            0x0f, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00
        };
    public:
        void init();
        volatile double degToSignal(double pos);
        char * setPosition(int pos[]);
        bool sendPosition(char* encodedPos);
};

#endif