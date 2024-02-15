#define GPIO1  12
#define PI 3.1415926535897932384626433832795

const int PID_SAMPLING_TIME = 100000; //in microseconds
const float PULSE_PER_REVOLUTION = 40; //no. of ticks arriving for 1 shaft revolution

const int required_rpm = 80;
int encoder_count = 0;
unsigned long last_reset_time;

//setting pins
int motorPin1 = 26;
int motorPin2 = 27;
int enablePin = 14;


const int freq = 60000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 50; // Approx half the voltage 1.65

void IRAM_ATTR isr()
{
  encoder_count ++;
}

float get_omega()
{
  float omega = (float) encoder_count;
  float time = (micros()-last_reset_time);
  time /= 1000000; //microseconds to seconds
  omega /= time;
  omega /= PULSE_PER_REVOLUTION;
  omega *= 2*PI;

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

  last_reset_time = micros();
}

void loop()
{
  if ((micros()-last_reset_time) < PID_SAMPLING_TIME)
    return;

  float omega = get_omega();
  Serial.println(omega);
  
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
  if(omega<required_rpm){
    dutyCycle += 1;
  }

  if(omega > required_rpm){
    dutyCycle -= 1;
  }

  if(dutyCycle>=255)
  {
    dutyCycle=255;
  }

  if(dutyCycle<=0)
  {
    dutyCycle =0;
  }
  //Serial.println(dutyCycle);
  ledcWrite(pwmChannel,dutyCycle);
}