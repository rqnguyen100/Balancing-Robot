#ifndef _BALANCE_H // header guard
#define _BALANCE_H

class balancing{
    public:
        balancing();
        void vertical(float list[5]);

    public:
        int kpV, kiV, kdV;
        int controlOutput;

    private:
        #define ANGLE_MIN -27
        #define ANGLE_MAX 27

};

class imu{
    public:
        float pitchBuffer[10];
        float offset;
    
    /*Functions*/
    public:
        imu();
        void init();
        void getYawPitchRoll(float& y, float& p, float& r);
        void calibrate();

    private:
        void offsets();
};

#endif