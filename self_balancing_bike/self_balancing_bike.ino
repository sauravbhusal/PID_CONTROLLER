#include <PID_v1.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 21
#define GPIO1 12
#define GPIO2 19
#define PI 3.1415926535897932384626433832795

bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];


float yaw_angle = 0.0;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
float yaw;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



const int PID_SAMPLING_TIME = 20000; // in microseconds
const float PULSE_PER_REVOLUTION = 40; // no. of ticks arriving for 1 shaft revolution

//const int required_rpm = 60;
int encoder_count = 0;
unsigned long last_reset_time;

// setting pins
int enablePin = 14;
int enablePin2 = 27;
const int freq = 60000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 0; // Approx half the voltage 1.65

// PID constants
double kp = 0.15;  // Proportional gain
double ki = 5.5; // Integral gain
double kd = 0.003;  // Derivative gain

double input, output, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

float yaw_calc()
{
      // if programming failed, don't try to do anything
    if (!dmpReady) 
      return 1.1;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            // display yaw angle in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("Yaw: ");
            yaw = ypr[0] * 180/M_PI;
            Serial.println(yaw);
    }
  return yaw;
}


void IRAM_ATTR isr()
{
  encoder_count++;
}

// void IRAM_ATTR isr2()
// {
//   kp+=0.0001;
// }

float get_omega()
{
  float omega = (float)encoder_count;
  float time = (micros() - last_reset_time);
  time /= 1000000; // microseconds to seconds
  omega /= time;
  omega /= PULSE_PER_REVOLUTION;
  omega *= 2 * PI;

  last_reset_time = micros();
  encoder_count = 0;
  return omega;
}

void setup()
{
  //MPU
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


  // PID
  // Serial.begin(115200);
  pinMode(enablePin, OUTPUT);

  pinMode(GPIO1, INPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enablePin, pwmChannel);
  ledcAttachPin(enablePin2, pwmChannel2);
  attachInterrupt(GPIO1, isr, CHANGE);
  //attachInterrupt(GPIO2, isr2, HIGH);
  last_reset_time = micros();

  // Initialize PID
  input = yaw_angle;
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop()
{

  yaw_angle = yaw_calc();
  if ((micros() - last_reset_time) < PID_SAMPLING_TIME)
    return;

  float omega = get_omega();
  
  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print(omega);
  //Serial.print(" ");
  input = omega;

  myPID.Compute(); // Calculate PID output
  if(yaw_angle<0)
  {
    ledcWrite(pwmChannel, output);
  }
  else {
    ledcWrite(pwmChannel2, output);
  }
  Serial.print(" ");
  // Serial.print("output");
  Serial.println(output);
  // dutyCycle = constrain(output, 0, 255);
  // Serial.println(dutyCycle);

  ledcWrite(pwmChannel, output);
}