#include <Arduino.h>
#include "MotorDrive.h"

MotorDrive::MotorDrive(int M1, int M2, int PWM){
  m1 = M1;
  m2 = M2;
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pwm = PWM;
}
void MotorDrive::MoveForward(int S){
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  analogWrite(pwm, S);
}
void MotorDrive::MoveBackward(int S){
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  analogWrite(pwm, S);
}
void MotorDrive::Stop(){
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
}
