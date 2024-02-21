#include <PID_v1.h>

#define GPIO1 12
#define GPIO2 19
#define PI 3.1415926535897932384626433832795

const int PID_SAMPLING_TIME = 20000; // in microseconds
const float PULSE_PER_REVOLUTION = 40; // no. of ticks arriving for 1 shaft revolution

const int required_rpm = 60;
int encoder_count = 0;
unsigned long last_reset_time;

// setting pins
int motorPin1 = 26;
int motorPin2 = 27;
int enablePin = 14;

const int freq = 60000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 150; // Approx half the voltage 1.65

// PID constants
float kp = 0.15;  // Proportional gain
float ki = 0.8; // Integral gain
float kd = 0.0001;  // Derivative gain

float input, output, setpoint=required_rpm ;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

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
  Serial.begin(115200);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(GPIO1, INPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enablePin, pwmChannel);
  attachInterrupt(GPIO1, isr, CHANGE);
  //attachInterrupt(GPIO2, isr2, HIGH);
  last_reset_time = micros();

  // Initialize PID
  input = get_omega();
  setpoint = required_rpm;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop()
{
  if ((micros() - last_reset_time) < PID_SAMPLING_TIME)
    return;

  float omega = get_omega();

  // Serial.print("KP");
  Serial.print(kp);
  Serial.print(" ");
  // Serial.print("omega");

  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print(omega);
  //Serial.print(" ");
  input = omega;

  myPID.Compute(); // Calculate PID output
  Serial.print(" ");
  // Serial.print("output");
  Serial.println(output);
  // dutyCycle = constrain(output, 0, 255);
  // Serial.println(dutyCycle);

  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);

  ledcWrite(pwmChannel, output);
}
