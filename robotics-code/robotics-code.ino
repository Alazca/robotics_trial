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
Phil's Lab (YouTube)             -- Error Filter for MPU
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
float gyroErrorX, gyroErrorY, gyroErrorZ = 0.0;
float gyroSense = 131;
float thetaX, thetaY = 0.0;
float errorAngleX, errorAngleY = 0.0;
int disDelay = 350;

// MPU - Accel variables
float rawAx, rawAy, rawAz = 0.0;
float accErrorX, accErrorY, accErrorZ = 0.0;
float phiX, phiY = 0.0;
float accelSense = 16384;

// Time variables
float timeNow, timeThen, dt = 0.0;

// Math variables and PID
float proportional, integral, derivative = 0.0;
float kp = 1.00;     // Needs to be small
float ki = 1.00;     // Needs to be smallest
float kd = 10.00;     // Needs to be biggest
float toDeg = 180 / PI;
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
void MPUerror();



void setup(){
    Serial.begin(BAUD);
    mtr.attach(servoPin);
    MPUtest();
    MPUerror();
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
    // Raw accelerometer data communication
    Wire.beginTransmission(MPUaddr);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPUaddr, 6);
 
    // Converting the raw analogue values of the MPU based on datasheet specification
    rawAx = ((Wire.read() << 8 | Wire.read())) / accelSense;
    rawAy = ((Wire.read() << 8 | Wire.read())) / accelSense;
    rawAz = ((Wire.read() << 8 | Wire.read())) / accelSense;

    // Raw gyro data communication
    Wire.beginTransmission(MPUaddr);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPUaddr, 6);
    
    // Converting the raw analogue values of the MPU based on datasheet specification
    rawGx = ((Wire.read() << 8 | Wire.read())) / gyroSense;
    rawGy = ((Wire.read() << 8 | Wire.read())) / gyroSense;
    rawGz = ((Wire.read() << 8 | Wire.read())) / gyroSense;
}

void processMPU(){ 
    timeCtrl(); 
    thetaX = (rawGx - gyroErrorX) * dt;
    thetaY = (rawGy - gyroErrorY) * dt;
    phiX = (atan2(rawAy,         sqrt(pow(rawAx,2) + pow(rawAz,2))) * toDeg) + accErrorY;
    phiY = (atan2(-1 * rawAx,    sqrt(pow(rawAy,2) + pow(rawAz,2))) * toDeg) + accErrorY;
    finalAngleX = (0.96 * phiX) + (0.04 * thetaX);
    finalAngleY = (0.96 * phiY) + (0.04 * thetaY);
}

void timeCtrl(){
    timeThen = timeNow;
    timeNow = millis();
    dt = (timeNow - timeThen) / 1000;   // converts dt to seconds
}

void mtrctrl(){
    actual = setPoint - finalAngleY;
    servoPos = pid(map(finalAngleY, -90, 90, 0, 180));   // References use +/- 170000 for mapping but not sure why
    if (servoPos != oldServo){
      mtr.write(actual);
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

void MPUerror(){
    Serial.println("Calibrating MPU - Recording Error. . .");
    const int RANGE = 200;
    for(int i = 0; i < RANGE; i++){
      rawMPU();
      accErrorX = atan2(rawAy,         sqrt(pow(rawAx,2) + pow(rawAz,2))) * toDeg;
      accErrorY = atan2(-1 * rawAx,    sqrt(pow(rawAy,2) + pow(rawAz,2))) * toDeg;
      i++;
      delay(25);
    }
    accErrorX = accErrorX / RANGE;
    accErrorY = accErrorY / RANGE;
    for (int i = 0; i < RANGE; i++){
      rawMPU();
      gyroErrorX += rawGx; 
      gyroErrorY += rawGy; 
      gyroErrorZ += rawGz; 
      i++;
      delay(25);
    }
    gyroErrorX = gyroErrorX / RANGE;
    gyroErrorY = gyroErrorY / RANGE;
    gyroErrorZ = gyroErrorZ / RANGE;
    Serial.print("Acceleration Error in X= ");
    Serial.print(accErrorX);
    Serial.print("\tAcceleration Error in Y: ");
    Serial.print(accErrorY);
    Serial.print("\nGyro Error in X: ");
    Serial.print(gyroErrorX);
    Serial.print("\tGyro Error in Y: ");
    Serial.print(gyroErrorY);
    Serial.print("\tGyro Error in Z: ");
    Serial.println(gyroErrorZ);
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
