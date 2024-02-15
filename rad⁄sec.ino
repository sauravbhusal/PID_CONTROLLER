#define GPIO1  12
#define PI 3.1415926535897932384626433832795

const int PID_SAMPLING_TIME = 200000; //in microseconds
const float PULSE_PER_REVOLUTION = 20; //no. of ticks arriving for 1 shaft revolution

int encoder_count = 0;
unsigned long last_reset_time;

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
  pinMode(GPIO1, INPUT);
  attachInterrupt(GPIO1, isr, RISING);

  last_reset_time = micros();
}

void loop()
{
  if ((micros()-last_reset_time) < PID_SAMPLING_TIME)
    return;

  float omega = get_omega();
  Serial.println(omega);
}