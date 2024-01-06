// Simple demo of PowerBoard for + with Arduino Uno
// for detailed description of the hardware, please refer to
// https://bastelstube.rocci.net/projects/MBS27_Arduino-Power/Arduino-Power.html
//
//  (c) Martin's Bastelstube 2023

#include <Wire.h>
#include <PowerBoard.h>

PowerBoard  Board;

void setup() {
  // Initialize the I2C bus first
  Wire.begin();
  // then initialize the PowerBoard
  Board.begin ();
}

void loop() {
  delay (500);
  Board.setMotorDirection(OUT_MOT0, MOTOR_FWD);
  Board.setMotorSpeed(OUT_MOT0, MOTOR_HALF);
  delay (500);
  Board.setMotorDirection(OUT_MOT1, MOTOR_FWD);
  Board.setMotorSpeed(OUT_MOT1, MOTOR_FULL);
  delay (500);
  Board.setMotorDirection(OUT_MOT0, MOTOR_OFF);
  Board.setMotorSpeed(OUT_MOT0, MOTOR_STOP);
  delay (500);
  Board.setMotorDirection(OUT_MOT1, MOTOR_OFF);
  Board.setMotorSpeed(OUT_MOT1, MOTOR_STOP);
  delay (1000);
}
