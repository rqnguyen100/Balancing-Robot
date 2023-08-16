#include <Arduino.h>
#include <motor.h>
// #include <PinChangeInterrupt.h>

motor::motor(){
    // constructor
}

void motor::motorPinInit(){
    pinMode(AIN1, OUTPUT); 
    pinMode(BIN1, OUTPUT); 
    pinMode(PWMA_LEFT, OUTPUT); 
    pinMode(PWMB_RIGHT, OUTPUT); 
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
}

void motor::forward(int speed){
    digitalWrite(AIN1, LOW); analogWrite(PWMA_LEFT, speed);
    digitalWrite(BIN1, LOW); analogWrite(PWMB_RIGHT, speed);
}

void motor::backward(int speed){
    digitalWrite(STBY_PIN, HIGH);
    digitalWrite(AIN1, HIGH); analogWrite(PWMA_LEFT, speed);
    digitalWrite(BIN1, HIGH); analogWrite(PWMB_RIGHT, speed);
}

void motor::stop(){
    analogWrite(PWMA_LEFT, 0); analogWrite(PWMB_RIGHT, 0);
}

/*----------------------------------------*/

void motor::encoderPinInit(){
    /*Pin Interrupts to Count Pulses*/
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN),motor::wheelPulse, CHANGE);
}

unsigned long motor::s_EncoderCount = 0;

void motor::wheelPulse(){
    motor::s_EncoderCount++;
}