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

void timer2::init(int time){
    /*Initialize the Timer Interrupt*/
    MsTimer2::set(time,interrupt);
    MsTimer2::start();
}

void timer2::interrupt(){
    sei();
    balanced.getVelocity(); 
    pitch.dataProcess();
    balanced.vertical();

    timerX.interruptCount++;
    if(timerX.interruptCount > 4){
        timerX.interruptCount=0;
        balanced.velocityControl();
    }

    balanced.totalControl();
}

balancing::balancing(){
    // constructor
    kp_balanced = 35; kp_speed = .1;
    ki_balanced = 1; 
    kd_balanced = 10;
}

/*----------------------------------------*/

void balancing::getVelocity(){
    /*Determine if motor is rotating forward or backwards*/
    int speed = balanced.pwmSpeed;

    if (speed > 0){
        balancing::velocity += encoder.s_EncoderCount;
    }
    else if (speed < -0){
        balancing::velocity -= encoder.s_EncoderCount;
    }

    balancing::velocity = constrain(balancing::velocity, -25, 25);

    // Resert values
    encoder.s_EncoderCount = 0;
}

void balancing::vertical(){    
    /*Calculating Control Output*/
    float pV = kp_balanced * pitch.currentPitch;
    float iV = ki_balanced * (pitch.currentPitch + pitch.prevPitch);
    float dV = kd_balanced * (pitch.currentPitch - pitch.prevPitch);
    pitch.prevPitch = pitch.currentPitch;
    
    balanced.controlOutput = pV + iV + dV;
    balanced.controlOutput = constrain(balanced.controlOutput,-125,125);
}

void balancing::velocityControl(){
    double velocity = balanced.velocity;

    balanced.velocityOutput = kp_speed * (0 - velocity);
}

void balancing::totalControl(){
    /*Calculate Speed*/
    balanced.vertical();
    balanced.velocityControl();

    balanced.pwmSpeed = balanced.controlOutput + balanced.velocity;

    /*Actuate Motor*/
    if (balanced.pwmSpeed > 0){
        encoder.forward(balanced.pwmSpeed);
    }
    else if (balanced.pwmSpeed < 0){
        encoder.backward(-balanced.pwmSpeed);
    }
}

/*----------------------------------------*/

MPU6050 mpu;

void imu::init(){
    /*Join I2C Bus*/
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif

    /*Initialize MPU*/
    mpu.initialize();
    mpu.testConnection();
    offsets(); // gyro offsets

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
}

void imu::offsets(){
    /*Gyro Offsets*/
    mpu.setXGyroOffset(75);
    mpu.setYGyroOffset(-256);
    mpu.setZGyroOffset(82);
    mpu.setZAccelOffset(416);
}

void imu::calibrate(){
    int ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Looking for values to set (gx,gy,gz,az) to (0,0,0,16384)
    // Will input values into offsets()
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.println(az);
}