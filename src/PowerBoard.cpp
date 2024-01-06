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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>
#include "PowerBoard.h"

// Constructor
PowerBoard::PowerBoard(void)
{
  init ();
}

// Set up the internal variables
void PowerBoard::init(void)
{
  uint8_t out_loop;

  _digi_mirror = 0xFF;

  for (out_loop = OUT_MOT0; out_loop < OUT_PORTS; out_loop ++)
  {
    _out_pwm[out_loop] = MOTOR_STOP;
  }
  _mot_direction_mirror = 0x00;

  _out_pin[OUT_MOT0] = PIN_MOT0;
  _out_pin[OUT_MOT1] = PIN_MOT1;
  _out_pin[OUT_MOT2] = PIN_MOT2;
  _out_pin[OUT_MOT3] = PIN_MOT3;
  _out_pin[OUT_SHOT] = PIN_SHOT;
  _out_pin[OUT_AUX]  = PIN_AUX;
}

void PowerBoard::begin(void)
{
  uint8_t out_loop;

  // Initialize the motor direction register
  Wire.beginTransmission(I2C_ADDR_MOTOR);
  Wire.write (_mot_direction_mirror);
  Wire.endTransmission();

  // Initialize the digital input/output port
  Wire.beginTransmission(I2C_ADDR_DIGITAL);
  Wire.write (_digi_mirror);
  Wire.endTransmission();

  // Initialize the PWM output pins (for motor speed)
  for (out_loop = OUT_MOT0; out_loop < OUT_PORTS; out_loop ++)
  {
    pinMode (_out_pin[out_loop], OUTPUT);
    analogWrite (_out_pin[out_loop], _out_pwm[out_loop]);
  }

  // User LED is switched OFF for the beginning
  pinMode (PIN_LED, OUTPUT);
  digitalWrite (PIN_LED, HIGH);
}

// ##############################################################
// ##   Access funtions for power output ports (motors etc.)   ##
// ##############################################################

// Set the direction of one motor without changing its speed
void PowerBoard::setMotorDirection(uint8_t mot_num, uint8_t mot_dir)
{
  uint8_t mot_loop;
  uint8_t mot_work;
  uint8_t mot_mask;

  mot_dir &= MOTOR_DMASK;

  if (mot_num >= OUT_ALL_M)
  {
    mot_work  =  mot_dir;
    mot_work |= (mot_dir << 2);
    mot_work |= (mot_dir << 4);
    mot_work |= (mot_dir << 6);
    _mot_direction_mirror = mot_work;

    Wire.beginTransmission(I2C_ADDR_MOTOR);
    Wire.write (_mot_direction_mirror);
    Wire.endTransmission();
  }
  else if (mot_num <= OUT_MOT3)
  {
    mot_mask  = MOTOR_DMASK << (mot_num << 1);
    mot_work  = _mot_direction_mirror;
    mot_work &= ~mot_mask;
    mot_work |= (mot_dir << (mot_num << 1));
    _mot_direction_mirror = mot_work;

    Wire.beginTransmission(I2C_ADDR_MOTOR);
    Wire.write (_mot_direction_mirror);
    Wire.endTransmission();
  }
}

// Get the direction of one motor
uint8_t PowerBoard::getMotorDirection(uint8_t mot_num)
{
  uint8_t mot_dir = MOTOR_OFF;

  if (mot_num <= OUT_MOT3)
  {
    mot_dir = ((_mot_direction_mirror >> (mot_num << 1)) & MOTOR_DMASK);
  }

  return mot_dir;
}

// Set the speed of one (or several) motor(s) without changing the direction
void PowerBoard::setMotorSpeed(uint8_t mot_num, uint8_t mot_speed)
{
  uint8_t mot_loop;

  if (mot_num == OUT_ALL_M)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_MOT3; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], mot_speed);
      _out_pwm[mot_loop] = mot_speed;
    }
  }
  else if (mot_num == OUT_ALL_S)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_SHOT; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], mot_speed);
      _out_pwm[mot_loop] = mot_speed;
    }
  }
  else if (mot_num == OUT_ALL_X)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_AUX; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], mot_speed);
      _out_pwm[mot_loop] = mot_speed;
    }
  }
  else if (mot_num < OUT_PORTS)
  {
    analogWrite (_out_pin[mot_num], mot_speed);
    _out_pwm[mot_num] = mot_speed;
  }
}

// Get the current speed of one motor
uint8_t PowerBoard::getMotorSpeed(uint8_t mot_num)
{
  uint8_t mot_speed = MOTOR_IDLE;

  if (mot_num < OUT_PORTS)
  {
    mot_speed = _out_pwm[mot_num];
  }

  return mot_speed;
}

// Set the speed + direction of one (or several) motor(s)
void PowerBoard::setMotor(uint8_t mot_num, int mot_speed)
{
  uint8_t mot_dir;

  if (mot_speed > 0)
  {
	mot_dir = MOTOR_FWD;  uint8_t mot_dir;

	if (mot_speed > 255)
	{
	  mot_speed = 255;
	}
  }
  else if (mot_speed < 0)
  {
	mot_dir = MOTOR_REV;
	mot_speed = -mot_speed;
	if (mot_speed > 255)
	{
	  mot_speed = 255;
	}
  }
  else  /* mot_speed == 0 */
  {
	mot_dir = MOTOR_OFF;
  }

  setMotorDirection (mot_num, mot_dir);
  setMotorSpeed (mot_num, mot_speed);
}

// Get the speed + direction of a motor combined in one value
int PowerBoard::getMotor(uint8_t mot_num)
{
  int     mot_speed;
  uint8_t mot_dir;
  
  mot_dir   = getMotorDirection (mot_num);
  mot_speed = getMotorSpeed (mot_num);

  if (mot_dir == MOTOR_REV)
  {
	mot_speed = -mot_speed;
  }

  return mot_speed;
}

void PowerBoard::stopMotor(uint8_t mot_num)
{
  uint8_t mot_loop;

  if (mot_num == OUT_ALL_M)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_MOT3; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], MOTOR_STOP);
      _out_pwm[mot_loop] = MOTOR_STOP;
    }
    _mot_direction_mirror = 0x00;
  }
  else if (mot_num == OUT_ALL_S)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_SHOT; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], MOTOR_STOP);
      _out_pwm[mot_loop] = MOTOR_STOP;
    }
    _mot_direction_mirror = 0x00;
  }
  else if (mot_num == OUT_ALL_X)
  {
    for (mot_loop = OUT_MOT0; mot_loop <= OUT_AUX; mot_loop ++)
    {
      analogWrite (_out_pin[mot_loop], MOTOR_STOP);
      _out_pwm[mot_loop] = MOTOR_STOP;
    }
    _mot_direction_mirror = 0x00;
  }
  else if (mot_num < OUT_PORTS)
  {
    analogWrite (_out_pin[mot_num], MOTOR_STOP);
    _out_pwm[mot_num] = MOTOR_STOP;
    _mot_direction_mirror &= ~(MOTOR_DMASK << (mot_num << 1));
  }

  Wire.beginTransmission(I2C_ADDR_MOTOR);
  Wire.write (_mot_direction_mirror);
  Wire.endTransmission();
}

// #########################################################################
// ##   Access functions for the digital input/output port via PCF8574A   ##
// #########################################################################

// Set one single bit of the digital output port to the specified value
void PowerBoard::setDigiBit(uint8_t bitpos, uint8_t bitval)
{
  bitpos &= 0x07;

  if (bitval == 0)
  {
    _digi_mirror &= ~(0x01 << bitpos);
  }
  else
  {
    _digi_mirror |= (0x01 << bitpos);
  }

  Wire.beginTransmission(I2C_ADDR_DIGITAL);
  Wire.write (_digi_mirror);
  Wire.endTransmission();
}

// Read one single bit from the internal mirror register
uint8_t PowerBoard::getDigiBit(uint8_t bitpos)
{
  uint8_t bitval;

  bitpos &= 0x07;

  if ((_digi_mirror & (0x01 << bitpos)) != 0)
  {
    bitval = BITVAL_HIGH;
  }
  else
  {
    bitval = BITVAL_LOW;
  }

  return bitval;
}

// Read one single bit from the digital input port
// (requires that the same bit has been set to HIGH before)
uint8_t PowerBoard::readDigiBit(uint8_t bitpos)
{
  uint8_t bitval;
  uint8_t i2c_data = 0xFF;

  bitval  = BITVAL_ERROR;
  bitpos &= 0x07;

  if (Wire.requestFrom(I2C_ADDR_DIGITAL, (uint8_t)1) == 1)
  {
    i2c_data = Wire.read();

    if ((i2c_data & (0x01 << bitpos)) != 0)
    {
      bitval = BITVAL_HIGH;
    }
    else
    {
      bitval = BITVAL_LOW;
    }
  }

  return bitval;
}

// Write all 8 bits of the digital output port at once
void PowerBoard::writeDigiPort(uint8_t digi_byte)
{
  _digi_mirror = digi_byte;

  Wire.beginTransmission(I2C_ADDR_DIGITAL);
  Wire.write (_digi_mirror);
  Wire.endTransmission();
}

// Read all 8 bits of the digital input port at once
uint8_t PowerBoard::readDigiPort(void)
{
  uint8_t digi_byte = 0xFF;

  if (Wire.requestFrom(I2C_ADDR_DIGITAL, (uint8_t)1) == 1)
  {
    digi_byte = Wire.read();
  }

  return digi_byte;
}

// ##############################################################################
// ##   Access function for measuring the battery voltage                      ##
// ##   This works only on analog input pin 3 if jumper JP2 is set correctly   ##
// ##############################################################################

// Measuring the battery voltage
// The return value is actually in millivolts !
// Readings below 7500 mV are inaccurate because the analog reference voltage is unreliable !
uint16_t PowerBoard::getBattVolt(void)
{
  uint16_t batt_raw;
  uint16_t batt_mvolt;
  float    batt_float;
  float    corr_factor =  24.024;  /* Correction factor may need fine tuning on each individual PowerBoard */
  float    corr_offset = 785.000;  /* Correction offset may need fine tuning on each individual PowerBoard */

  batt_raw   = analogRead (PIN_BATT_VOLT);      /* Raw reading (10 bit) is 0 .. 1023 */
  batt_float = (batt_raw * corr_factor) + corr_offset;
  batt_mvolt = int (batt_float);

  return batt_mvolt;
}
