#ifndef _BALANCE_H // header guard
#define _BALANCE_H

class balancing{
    public:
        balancing();
        void getVelocity();
        void velocityControl();

        void vertical();
        void totalControl();

    public:
        int kp_balanced, ki_balanced, kd_balanced;
        int kp_speed;
        int controlOutput;

        int velocity;
        int velocityOutput;
        
        int pwmSpeed = 0;

    private:
        #define ANGLE_MIN -27
        #define ANGLE_MAX 27

};

class timer2{
    public:
        int interruptCount;
    
    public:
        void init(int time);
        static void interrupt();
    
    private:       
          #define TIMER 5
};

class imu{
    public:
        float offset;
        float pitchBuffer;
        float currentPitch;
        float prevPitch = 0;
        int trials = 10;
    
    /*Functions*/
    public:
        void init();
        void getYawPitchRoll(float& y, float& p, float& r);
        void dataProcess();
        void calibrate();

    private:
        void offsets();
};

#endif