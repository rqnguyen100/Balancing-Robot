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
    Serial.println("Initing...");

    /*Pin Initializers*/
    c_Motor.motorPinInit(); 
    c_Motor.encoderPinInit();

    /*Initialize MPU-6050*/    
    c_MPU6050.init();

    Serial.println("Ready");

    /*Timer Initalize*/
    timer.init(TIMER);}

void loop(){
}