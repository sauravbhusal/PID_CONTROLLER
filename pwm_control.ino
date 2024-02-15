int motorPin1 = 26;
int motorPin2 = 27;
int enablePin = 14;

const int freq = 60000;
const int pwmChannel = 0;
const int resolution = 8;
const int sensorPin = 12;
int dutyCycle = 127; // Approx half the voltage 1.65

unsigned long start_time = 0;
unsigned long end_time = 0;
int steps = 0;
float steps_old = 0;
float temp = 0;
float rps = 0 ;
float rpm = 0;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(sensorPin, INPUT);

  ledcSetup(pwmChannel, freq, resolution);

  ledcAttachPin(enablePin, pwmChannel);

  Serial.begin(115200);
}

void loop() {

  start_time = millis();
  end_time = start_time+1000;

  //measure steps taken within 1 second
  while (millis()<end_time){
    if (digitalRead(sensorPin)){
      steps = steps+1;
      while(digitalRead(sensorPin)); /* Until next pulse*/
    }
  }
  temp = steps-steps_old;
  steps_old = steps;
  rps = temp/20;
  rpm = rps*60;
  ets_print("rpm:%f\n", rpm);
  
  //move the motor in increasing speed
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
  ledcWrite(pwmChannel,dutyCycle);
}
