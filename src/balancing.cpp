#include <Arduino.h>
#include <balancing.h>
#include <motor.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "MsTimer2.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

balancing balanced;
imu pitch;
motor encoder;
timer2 timerX;
MPU6050 mpu;

void timer2::init(int time){
    /*Initialize the Timer Interrupt*/
    MsTimer2::set(time,interrupt);
    MsTimer2::start();
}

void timer2::interrupt(){
    sei();
    balanced.getAcceleration(); 
    pitch.dataProcess();
    balanced.vertical();

    timerX.interruptCount++;
    if(timerX.interruptCount > 8){
        timerX.interruptCount=0;
        balanced.accelerationControl();
    }

    balanced.totalControl();
}

balancing::balancing(){
    // constructor
    kp_balanced = 60; kp_acc = 10;
    ki_balanced = 1; 
    kd_balanced = 15;
}

/*----------------------------------------*/

void balancing::getAcceleration(){
    /*Determine if motor is rotating forward or backwards*/
    int speed = balanced.pwmSpeed;
    int iDeltaT = 1000/40;

    if (speed > 0){
        balanced.currentPos += (2 * 3.14 * 7 * encoder.s_EncoderCount / 658); // cm
    }
    else if (speed < -0){
        balanced.currentPos -= (2 * 3.14 * 7 * encoder.s_EncoderCount / 658); // cm
    }

    balanced.currentVel = (balanced.currentPos - balanced.prevPos) * iDeltaT; 
    balanced.currentAcc = (balanced.currentVel - balanced.prevVel) * iDeltaT; 

    /*High Pass Filter*/
    float weight = 0.5; 
    balanced.filteredAcc = weight * (balanced.filteredAcc + balanced.currentAcc - balanced.prevAcc);

    // Reset values
    encoder.s_EncoderCount = 0;
    balanced.prevPos = balanced.currentPos; balanced.prevVel = balanced.currentVel;
    balanced.prevAcc = balanced.currentAcc;
}

void balancing::vertical(){    
    /*Calculating Control Output*/
    float pV = kp_balanced * pitch.currentPitch;
    float iV = ki_balanced * (pitch.currentPitch + pitch.prevPitch);
    float dV = kd_balanced * (pitch.currentPitch - pitch.prevPitch);
    pitch.prevPitch = pitch.currentPitch;
    
    balanced.controlOutput = pV + iV + dV;
    balanced.controlOutput = constrain(balanced.controlOutput, -250, 250);
}

void balancing::accelerationControl(){
    double acc = balanced.filteredAcc;

    balanced.accelerationOutput = kp_acc * -acc;
    balanced.accelerationOutput = constrain(balanced.accelerationOutput, -100, 100);
}

void balancing::totalControl(){
    /*Calculate Speed*/
    balanced.vertical();
    balanced.accelerationControl();

    /*Complimentary Filter*/
    float weight = 0.5;
    balanced.pwmSpeed = weight * balanced.controlOutput + (1-weight) * balanced.accelerationOutput;
    balanced.pwmSpeed = constrain(balanced.pwmSpeed,-150,150);

    /*Actuate Motor*/
    if (balanced.pwmSpeed > 0){
        encoder.forward(balanced.pwmSpeed);
    }
    else if (balanced.pwmSpeed < 0){
        encoder.backward(-balanced.pwmSpeed);
    }
}

/*----------------------------------------*/

void imu::init(){
    /*Join I2C Bus*/
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif

    /*Initialize MPU*/
    mpu.initialize();
    mpu.testConnection();
    offsets(); // offsets

    /*Initilize DMP*/
    uint8_t devStatus;
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0){
        mpu.setDMPEnabled(true);
    }

}

void imu::getYawPitchRoll(float& y, float& p, float& r){
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z] quaternion container

    /*Calculate YPR from Latest Packet*/
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        float q0 = q.w; float q1 = q.x; float q2 = q.y; float q3 = q.z;

        float yr = -atan2(-2*q1*q2 + 2*q0*q3, q2*q2 - q3*q3 - q1*q1 + q0*q0);
        float pr = asin(2*q2*q3 + 2*q0*q1);
        float rr = atan2(-2*q1*q3 + 2*q0*q2, q3*q3 - q2*q2 - q1*q1 + q0*q0);

        y = yr * 180 / M_PI;
        p = pr * 180 / M_PI;
        r = rr * 180 / M_PI;
    }
}

void imu::dataProcess(){
    /*Get MPU Data*/
    float yaw, pitch, roll;
    float offset = -5.5;

    imu::getYawPitchRoll(yaw, pitch, roll);
    imu::pitchBuffer = 0;
    for (int i = 0; i < imu::trials; i++){
        imu::pitchBuffer += (pitch - offset);
    }
    imu::currentPitch = pitchBuffer / imu::trials;

    /*Low Pass Filter*/
    float weight = 0.95;
    imu::currentPitch = weight * imu::currentPitch + (1 - weight) * imu::prevPitch;
}

void imu::calibrate(){
    int ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Looking for values to set to (0,0,0,0,0,16384)
    // Will input values into offsets()
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.println(gz);
}

void imu::offsets(){
    /*Gyro Offsets*/
    mpu.setXAccelOffset(-2974);
    mpu.setYAccelOffset(-2335);
    mpu.setZAccelOffset(1503);
    mpu.setXGyroOffset(138);
    mpu.setYGyroOffset(-502);
    mpu.setZGyroOffset(157);
}