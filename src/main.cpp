#include <Arduino.h>
#include <motor.h> 
#include <balancing.h>

/*Classes*/
timer2 timer;
motor c_Motor;
imu c_MPU6050;
balancing c_Balanced;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing");

    /*Pin Initializers*/
    c_Motor.motorPinInit(); 
    c_Motor.encoderPinInit();
    Serial.println("Pins Initialized");

    /*Initialize MPU-6050*/    
    c_MPU6050.init();
    Serial.println("MPU Initialized");

    /*Timer Initalize*/
    Serial.println("Ready");
    timer.init(TIMER);
}

void loop(){
}