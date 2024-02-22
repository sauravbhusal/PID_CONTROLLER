//for pid
#include <Wire.h>
#include <Arduino.h>
#include <PID_v1.h>

//for  mpu
#include <I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>

// for pid
const int PID_SAMPLING_TIME = 20000; // in microseconds
const float PULSE_PER_REVOLUTION = 40; 

float deltaT = 0, currentTime = 0, previousTime = 0;

int encoder_count = 0;
unsigned long last_reset_time;

double kp = 40; //
double ki = 0; //.14; //
double kd = 0; //

double target_angle = 0;
double input,output;
double setpoint = target_angle;
PID myPID(&input,&output,&setpoint, kp, ki,kd,DIRECT);

int pwm_calc; 
int pwm_output;
float angle_bounce = 0.1;
int i = 0;

//for mpu
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 21

float roll_angle = 0.0;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];    

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//
const int rPwmPin = 14;  // Replace with the GPIO pin connected to the R_PWM input of HW039
const int lPwmPin = 27;
const int pwmChannelR = 0;  // PWM channel for R_PWM
const int pwmChannelL = 1;  // PWM channel for L_PWM
const int frequencyR = 10000;  // PWM frequency for R_PWM in Hz


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

        Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-32);
    mpu.setYGyroOffset(-26);
    mpu.setZGyroOffset(32);
    mpu.setZAccelOffset(3083);
    mpu.setYAccelOffset(-947);
    mpu.setXAccelOffset(-5047);

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
       } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
  //
  int resolution = 8;
  pinMode(rPwmPin, OUTPUT);
  pinMode(lPwmPin, OUTPUT);
  
  // Configure PWM for R_PWM
  ledcSetup(pwmChannelR, frequencyR, resolution);  // 8-bit resolution for PWM
  ledcAttachPin(rPwmPin, pwmChannelR);

  // Configure PWM for L_PWM
  ledcSetup(pwmChannelL, frequencyR, resolution);  // 8-bit resolution for PWM
  ledcAttachPin(lPwmPin, pwmChannelL);

  //for pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
}

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  deltaT = ((currentTime - previousTime) / 1000); // Divide by 1000 to get seconds

  

  //mpu
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
       
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.println(ypr[2] * 180/M_PI);
           
        #endif
    }
    roll_angle = ypr[2] * 180/M_PI;
    input = roll_angle;
  //
  if( i  > 0){
    if(roll_angle < target_angle){
      target_angle -= angle_bounce * deltaT;
    }
    else if(roll_angle > target_angle){
      target_angle += angle_bounce * deltaT;
    }
    else{
      target_angle = 0;
    }
  }
  myPID.Compute();
  // Serial.print("output = ");
  // Serial.println(output);
  pwm_calc = constrain(output, -255, 255);
  // Serial.print("pwm = ");
  
  // Serial.println(pwm_calc);
    if(pwm_calc <=0){
      if (abs(roll_angle)>25){
      pwm_output = 255;
      }
      else{
       pwm_output = abs(pwm_calc);
        run_reaction_right(pwm_output);
      }
    }
    
    else if (pwm_calc > 0){
      if (abs(roll_angle)>25){
        pwm_output = 255;
      }
      else{
        pwm_output = abs(pwm_calc);
      }
      run_reaction_left(pwm_output);

    }
    i += 1;
}

void setRPWMDutyCycle(uint8_t dutyCycle, int pwmChannel) {
  ledcWrite(pwmChannel, dutyCycle);
}

void run_reaction_right(int pwm_output) {
  setRPWMDutyCycle(pwm_output, pwmChannelR);  // 50% duty cycle (assuming 8-bit resolution)

}

void run_reaction_left(int pwm_output) {
  setRPWMDutyCycle(pwm_output, pwmChannelL);  // 50% duty cycle (assuming 8-bit resolution)

}