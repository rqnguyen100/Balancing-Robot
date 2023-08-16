#ifndef _MOTOR_H // header guard
#define _MOTOR_H

class motor{

    public:
        /*Variables*/
        static unsigned long s_EncoderCount;

    public:
        /*Functions*/
        motor();

        void motorPinInit();
        void encoderPinInit();
        
        void forward(int speed);
        void backward(int speed);
        void stop();

        static void wheelPulse();

    private:
        /*Motor pin*/
        #define AIN1 7
        #define PWMA_LEFT 5 
        #define BIN1 12 
        #define PWMB_RIGHT 6 
        #define STBY_PIN 8

        /*Encoder measuring speed pin*/ 
        #define ENCODER_LEFT_A_PIN 2 
};

#endif