#include <Arduino.h>
#include <motor.h> 
#include <balancing.h>

/*Classes*/
motor c_Motor;
imu c_MPU6050;
balancing c_Balanced;

void setup() {
    Serial.begin(115200);

    /*Pin Initializers*/
    c_Motor.motorPinInit(); 
    // c_Motor.encoderPinInit();

    /*Initialize MPU-6050*/    
    c_MPU6050.init();
}

void loop(){
    /*Get MPU Data*/
    float yaw, pitch, roll;
    float offset = -5;
    for (int i = 0; i < 5; i++){
        c_MPU6050.getYawPitchRoll(yaw, pitch, roll);
        c_MPU6050.pitchBuffer[i] = pitch - offset;
    }

    /*Get Motor Output Based on PID*/
    c_Balanced.vertical(c_MPU6050.pitchBuffer);
    int speed = c_Balanced.controlOutput;
    speed = constrain(speed,-150,150);

    /*Actuate Motor*/
    if (speed > 0){
        c_Motor.forward(speed);
    }
    else if (speed < 0){
        c_Motor.backward(-speed);
    }

    /*Stop Motor if Excessive Tilting*/
    float tilt = c_MPU6050.pitchBuffer[4];
    Serial.print(tilt);
    Serial.print("\t");
    Serial.println(speed);
    
    if (tilt > ANGLE_MAX || tilt < ANGLE_MIN){
        c_Motor.stop();
    }

    delay(40);
}