/***************************
Robotics Trial Project
by 
Andy Lazcano

Due to being an ammature in microcontrollers and coding low level, I'd like to thank the following places for inspiration 
and thank them for their guidance.

Paul McWhorter (YouTube)         -- General Arduino / Servo help
howtomechanics.com               -- MPU6050, calcuating roll, pitch, yaw
forum.arduino.cc                 -- General help with pinout / specifications
Ian Carey (YouTube)              -- Integrating PID to arduino codebase 
Electronoobs (YouTube)           -- General microcontrollers

******************************/
#include <Wire.h>
#include <Servo.h>
#include <math.h>

#define MPUaddr 0x68
#define BAUD 9600

// Servo constants
Servo mtr;
float servoPos = 0.0;  // the angle of the servo
float oldServo = 0.0;
float setPoint = 0.0;   // Desired sv
float actual = 0.0;
int servoPin = 5;
int servoDelay = 15;
int loopDelay = 10;

// MPU - Gyro variables 
float rawGx, rawGy, rawGz = 0.0;
float gyroSense = 131;
float thetaX, thetaY = 0.0;
float errorAngleX, errorAngleY = 0.0;
int disDelay = 350;

// MPU - Accel variables
float rawAx, rawAy, rawAz = 0.0;
float phiX, phiY = 0.0;
float accelSense = 16384;

// Time variables
float timeNow, timeThen, dt = 0.0;

// Math variables and PID
float proportional, integral, derivative = 0.0;
float kp = 0.20;     // Needs to be small
float ki = 0.03;     // Needs to be smallest
float kd = 25.0;     // Needs to be biggest
float toDeg = 180 * PI;
float error, oldError, corrected = 0.0;
float finalAngleX, finalAngleY = 0.0;

// Prototypes
float pid(float);
void MPUtest();
void servoTest();
void rawMPU();
void processMPU();
void mtrctrl();
void MPUdisplay();

void setup(){
    Serial.begin(BAUD);
    mtr.attach(servoPin);
    MPUtest();
    servoTest();
}

void loop(){
    rawMPU();
    processMPU();
    mtrctrl();
    MPUdisplay();
    delay(loopDelay);
}

void MPUtest(){
    Wire.beginTransmission(MPUaddr);        // I2C address of the mpu in standard, slave setting
    Wire.write(0x6B);                       // Datasheet
    Wire.write(0x00);
    Wire.endTransmission();
    Serial.println("MPU Test Complete!");
}

void servoTest(){
    // Performs sweep test. 
    mtr.write(0);
    for (servoPos = 0; servoPos <= 180; servoPos += 1){
        mtr.write(servoPos);
        delay(servoDelay);
    }
    for (servoPos = 180; servoPos >= 0; servoPos -= 1){
        mtr.write(servoPos);
        delay(servoDelay);
    }
    Serial.println("Servo Test Complete!");
}

void rawMPU(){
    // Raw gyro data communication
    Wire.beginTransmission(MPUaddr);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPUaddr, 6);
    
    // Converting the raw analogue values of the MPU based on datasheet specification
    rawGx = ((Wire.read() << 8 | Wire.read())) / gyroSense;
    rawGy = ((Wire.read() << 8 | Wire.read())) / gyroSense;
    rawGz = ((Wire.read() << 8 | Wire.read())) / gyroSense;
  
    // Raw accelerometer data communication
    Wire.beginTransmission(MPUaddr);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPUaddr, 6);
 
    // Converting the raw analogue values of the MPU based on datasheet specification
    rawAx = ((Wire.read() << 8 | Wire.read())) / accelSense;
    rawAy = ((Wire.read() << 8 | Wire.read())) / accelSense;
    rawAz = ((Wire.read() << 8 | Wire.read())) / accelSense;
}

void processMPU(){ 
    timeCtrl(); 
    thetaX = rawGx * dt;
    thetaY = rawGy * dt;
    phiX = atan2(rawAy,         sqrt(pow(rawAx,2) + pow(rawAz,2))) * toDeg;
    phiY = atan2(-1 * rawAx,    sqrt(pow(rawAy,2) + pow(rawAz,2))) * toDeg;
    finalAngleX = (phiX + thetaX);
    finalAngleY = (phiY + thetaY);
}

void timeCtrl(){
    timeThen = timeNow;
    timeNow = millis();
    dt = (timeNow - timeThen) / 1000;   // converts dt to seconds
}

void mtrctrl(){
    actual = setPoint - finalAngleX;
    servoPos = pid(map(finalAngleX, -90, 90, 0, 180));   // References use +/- 170000 for mapping but not sure why
    if (servoPos != oldServo){
      mtr.write(servoPos);
      oldServo = servoPos;
    }
    oldError = actual;
}

void MPUdisplay(){
    Serial.print("Angle X = ");
    Serial.print(finalAngleX);
    Serial.print("\t, Angle Y = ");
    Serial.print(finalAngleY);
    Serial.print(", Gyro Z = ");
    Serial.println(rawGz);
    delay(disDelay);
}

float pid(float error){
    timeCtrl();
    proportional = error;
    integral += error * dt;
    derivative = (error - oldError) / dt;     // dx/dt
    oldError = error;
    corrected = (kp * proportional) + (ki * integral) + (kd * derivative);
    return corrected;
}
