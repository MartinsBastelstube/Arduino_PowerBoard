/*****************************************************************/
/**                                                             **/
/**   Arduino library for PowerBoard by Martin's Bastelstube    **/
/**    with four DC motor outputs, one  high-current output     **/
/**    and one auxiliary output, each of them PWM controlled,   **/
/**    eight digital input/output pins via PCF8574A,            **/
/**    up to four analog input pins, of which one can be        **/
/**    configured to measure the battery voltage.               **/
/**                                                             **/
/*************************************************************************************/
/**                                                                                 **/
/**  https://bastelstube.rocci.net/projects/MBS27_Arduino-Power/Arduino-Power.html  **/
/**                                                                                 **/
/*************************************************************************************/

#ifndef POWERBOARD_H
#define POWERBOARD_H

#include <inttypes.h>

// Power output port IDs
#define OUT_MOT0   0
#define OUT_MOT1   1
#define OUT_MOT2   2
#define OUT_MOT3   3
#define OUT_SHOT   4
#define OUT_AUX    5
#define OUT_PORTS  6  // Number of PWM-controlled output ports
#define OUT_ALL_M 10  // valid for all motor outputs
#define OUT_ALL_S 11  // valid for all outputs except AUX
#define OUT_ALL_X 12  // valid for all outputs including AUX

// PWM control pins for power output ports
#define PIN_MOT0   3
#define PIN_MOT1   5
#define PIN_MOT2   6
#define PIN_MOT3   9
#define PIN_SHOT  10
#define PIN_AUX   11

// On-board LED for whatever purpose you may choose
#define PIN_LED    8

// Analog input pins
#define PIN_BATT_VOLT  3

// Predefined values for motor speed
#define MOTOR_IDLE  0x00
#define MOTOR_STOP  0x00
#define MOTOR_WEAK  0x10
#define MOTOR_SLOW  0x20
#define MOTOR_QUART 0x40
#define MOTOR_HALF  0x80
#define MOTOR_FULL  0xFF

// Predefined values for motor direction
#define MOTOR_OFF   0
#define MOTOR_FWD   1
#define MOTOR_REV   2
#define MOTOR_DMASK 3  // Bitmask for filtering the motor direction

// I2C device addresses
#define I2C_ADDR_MOTOR    0x26  // PCF8574  for motor directions
#define I2C_ADDR_DIGITAL  0x3E  // PCF8574A for digital I/O port

// Return value from readDigiBit
#define BITVAL_LOW       0
#define BITVAL_HIGH      1
#define BITVAL_ERROR     3

class PowerBoard {
public:
  PowerBoard(void);
  void init(void);
  void begin(void);

  // Controlling the power output ports (motors etc.)
  void    setMotorDirection(uint8_t, uint8_t);
  uint8_t getMotorDirection(uint8_t);
  void    setMotorSpeed(uint8_t, uint8_t);
  uint8_t getMotorSpeed(uint8_t);
  void    setMotor(uint8_t, int);
  int     getMotor(uint8_t);
  void    stopMotor(uint8_t);

  // Reading / writing the digital input/output port
  void    setDigiBit(uint8_t, uint8_t);
  uint8_t getDigiBit(uint8_t);
  uint8_t readDigiBit(uint8_t);
  void    writeDigiPort(uint8_t);
  uint8_t readDigiPort(void);

  // Measuring the battery voltage
  // works only on analog input pin 3 if jumper JP2 is set correctly
  uint16_t getBattVolt(void);

private:

  // internal variables
  uint8_t _out_pin[OUT_PORTS];
  uint8_t _out_pwm[OUT_PORTS];
  uint8_t _mot_direction_mirror;
  uint8_t _digi_mirror;
};

#endif   /* POWERBOARD_H */
