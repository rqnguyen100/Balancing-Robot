#include <Arduino.h>
#include <balancing.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

balancing::balancing(){
    // constructor
    kpV = 1;
    kiV = 0; 
    kdV = 0;
}

void balancing::vertical(float list[5]){
    /*Finding Integral Term*/
    float sum = 0;
    for (int i = 0; i < 5; i++){
        sum += list[i];
    }

    float pV = kpV * list[4];
    float iV = kiV * sum;
    float dV = kdV * (list[4] - list[3]);

    balancing::controlOutput= pV + iV + dV;
}

/*----------------------------------------*/

MPU6050 mpu;

imu::imu(){
    // constructor
}

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