#include <QTRSensors.h>
#define sensor_direita 23

// Sensores IR - Preto = 1; Branco = 0;

int counter = 0;
int leitura_direita;
bool running = true;
QTRSensors qtr;

#define IR 8
unsigned long previoustime = 0;
unsigned short qtrValues[IR];
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// 5500 - 6500 (carro pra fora à direita) virar esquerda
// 500 - 1500 (carro para fora à esquerda) virar direita

#define Kp 9    // 255: 0.1     110: 0.2
#define Ki 0    // 255: 0.05    110: 0.05
#define Kd 3.5   // 255: 0.003   110: 0.004
#define rightMaxSpeed 255  // 255  50
#define leftMaxSpeed  255  // 255  50

int SetPoint = (IR - 1) * 500;
 
// Motor-A
int IN1 = 16;
int IN2 = 17;

// Motor-B
int IN3 = 19;
int IN4 = 21;

int lastError=0;
unsigned long cTime, pTime;
float eTime;
float P_error;
float I_error;
float D_error;
float PID_value;

void setup() {
  Serial.begin(115200);  
  qtr.setTypeRC();


  qtr.setSensorPins((const uint8_t[]){4, 12, 14, 27, 26, 25, 33, 32},IR);
  qtr.setEmitterPin(35);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  pinMode(IN1, OUTPUT);   
  pinMode(IN2, OUTPUT);   
  pinMode(IN3, OUTPUT);   
  pinMode(IN4, OUTPUT);
  pinMode(sensor_direita, INPUT);
  pinMode(sensor_esquerda, INPUT);

  for (int i = 0; i < 500; i++){
    delay(20);
  }
}

void right_brake() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 255);  
  analogWrite(IN3, 255);
  analogWrite(IN4, 0);  
}

void left_brake() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);  
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);  
}

void run_fwd(int valueSA, int valueSB){
    analogWrite(IN1, 0);
    analogWrite(IN2, valueSB);  
    analogWrite(IN3, 0);
    analogWrite(IN4, valueSA);  
}

void stop_motor() {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);  
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
}

void side_sensors() {
  leitura_direita = digitalRead(sensor_direita);
  leitura_esquerda = digitalRead(sensor_esquerda);

  if (counter >= 4) {
    running = false;
  }
  if ((!leitura_direita) && (millis() - previoustime >= 400)) {
    previoustime = millis();
    counter++;
  }

}
void Run_robot(){
  lastError = P_error;
  pTime = cTime;
  int position = qtr.readLineWhite(qtrValues);

  side_sensors();

  if (position >= 3300 && position <= 3700) {
    run_fwd(255, 255);
  }

  else if (position >= 5500) {
    left_brake();
  }

  else if (position >= 50 && position <= 2500) {
    right_brake();
  }

  else {
    int med_Speed_R;
    int med_Speed_L;
    P_error = position - SetPoint;
    cTime = millis();
    eTime = (float)(cTime - pTime) / 1000;
    I_error = I_error * 2 / 3 + P_error * eTime;
    D_error = (P_error - lastError) / eTime;
    PID_value = Kp * P_error + Ki * I_error + Kd * D_error;
    med_Speed_L = leftMaxSpeed - abs(PID_value);
    med_Speed_R = rightMaxSpeed - abs(PID_value);
    int leftMotorSpeed = med_Speed_L + PID_value;
    int rightMotorSpeed = med_Speed_R - PID_value;
    leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);

    //Serial.print(rightMotorSpeed);
    //Serial.print("   ");
    //Serial.println(leftMotorSpeed);

    run_fwd(leftMotorSpeed, rightMotorSpeed);
    delayMicroseconds(140);

  }
}

void loop() {
  //
  //Serial.println(digitalRead(counter));

  while(running) {
    Run_robot();
    side_sensors();
  }

  stop_motor();

  //Serial.println(digitalRead(sensor_1));

}