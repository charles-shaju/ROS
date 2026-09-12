#include <Arduino.h>

// --- Pin Definitions ---
const int PIN_PWM_A = 2;   
const int PIN_AIN1 = 7;    
const int PIN_AIN2 = 10;   

const int PIN_PWM_B = 8;   
const int PIN_BIN1 = 0;    
const int PIN_BIN2 = 1;    

const int PIN_ENC_A1 = 3;  
const int PIN_ENC_A2 = 4;  
const int PIN_ENC_B1 = 5;  
const int PIN_ENC_B2 = 6;  

// --- Volatile counters ---
volatile long encoder_a_ticks = 0;
volatile long encoder_b_ticks = 0;

// --- Target Speeds ---
int target_speed_a = 0;
int target_speed_b = 0;

void IRAM_ATTR handleEncoderA() {
  if (digitalRead(PIN_ENC_A2) == HIGH) {
    encoder_a_ticks++;
  } else {
    encoder_a_ticks--;
  }
}

void IRAM_ATTR handleEncoderB() {
  if (digitalRead(PIN_ENC_B2) == HIGH) {
    encoder_b_ticks++;
  } else {
    encoder_b_ticks--;
  }
}

void setMotorSpeeds(int speedA, int speedB) {
  // Motor A Control
  if (speedA >= 0) {
    analogWrite(PIN_PWM_A, speedA);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    analogWrite(PIN_PWM_A, -speedA);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }

  // Motor B Control
  if (speedB >= 0) {
    analogWrite(PIN_PWM_B, speedB);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    analogWrite(PIN_PWM_B, -speedB);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_ENC_A1, INPUT_PULLUP);
  pinMode(PIN_ENC_A2, INPUT_PULLUP);
  pinMode(PIN_ENC_B1, INPUT_PULLUP);
  pinMode(PIN_ENC_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A1), handleEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B1), handleEncoderB, RISING);
}

void loop() {
  // 1. Check for incoming serial commands from the Pi (Format: "SPEED_A,SPEED_B\n")
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      target_speed_a = input.substring(0, commaIndex).toInt();
      target_speed_b = input.substring(commaIndex + 1).toInt();
      setMotorSpeeds(target_speed_a, target_speed_b);
    }
  }

  // 2. Stream encoder telemetry back to the Pi (Format: "ENC:ticksA,ticksB\n")
  Serial.print("ENC:");
  Serial.print(encoder_a_ticks);
  Serial.print(",");
  Serial.println(encoder_b_ticks);

  delay(50); // 20Hz telemetry update rate
}