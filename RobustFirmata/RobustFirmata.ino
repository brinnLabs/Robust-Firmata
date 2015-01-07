/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please clink on the following link
 * to open the download page in your default browser.
 *
 * http://firmata.org/wiki/Download
 */

/*
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
 Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
 Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
 Copyright (C) 2009-2014 Jeff Hoefs.  All rights reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 See file LICENSE.txt for further informations on licensing terms.
 
 formatted using the GNU C formatting and indenting
 */

#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>


/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info
{
  byte addr;
  int reg;
  byte bytes;
};

/* for i2c read continuous more */
i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()

struct ow_device_info
{
  OneWire* device;
  byte addr[8];
  boolean power;
};

ow_device_info pinOneWire[TOTAL_PINS];

FirmataStepper *stepper[MAX_STEPPERS];
byte numSteppers = 0;

Encoder encoders[MAX_ENCODERS];
int32_t positions[MAX_ENCODERS];
int32_t prevPositions[MAX_ENCODERS];
byte numAttachedEncoders=0;
byte reportEncoders = 0x00;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS)
  {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0)
    {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    }
    else
    {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0)
    {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    }
    else
    {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  }
  else
  {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0)
  {
    servoCount--;
  }
  else if (servoCount > 0)
  {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

/**
 * TO-DO
 * ==============
 * Track encoders attaching and detatching, perhaps use a similar method to servos
 * 
 **/

void attachEncoder(byte encoderNum, byte pinANum, byte pinBNum)
{
  if (isEncoderAttached(encoderNum))
  {
    Firmata.sendString("Encoder Warning: encoder is already attached. Operation cancelled.");
    return;
  }

  if (!IS_PIN_INTERRUPT(pinANum) || !IS_PIN_INTERRUPT(pinBNum))
  {
    Firmata.sendString("Encoder Warning: For better performences, you should only use Interrput pins.");
  }
  setPinModeCallback(pinANum, ENCODER);
  setPinModeCallback(pinBNum, ENCODER);
  encoders[encoderNum] = Encoder(pinANum, pinBNum);
  numAttachedEncoders++;
  reportEncoderPosition(encoderNum);
}

/**
 * TO-DO
 * ==============
 * How should we detach them? now it just gives it an empty object, need a way to make it null
 * 
 **/

void detachEncoder(byte encoderNum)
{
  if (isEncoderAttached(encoderNum))
  {
    //free(encoders[encoderNum]);
    encoders[encoderNum] = Encoder();
    numAttachedEncoders--;
  }
}

void oneWireConfig(byte pin, boolean power){
  ow_device_info *info = &pinOneWire[pin];
  if (info->device==NULL) {
    info->device = new OneWire(pin);
  }
  for (int i=0;i<8;i++) {
    info->addr[i]=0x0;
  }
  info->power = power;
}

void readAndReportData(byte address, int theRegister, byte numBytes)
{
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != REGISTER_NOT_SPECIFIED)
  {
    Wire.beginTransmission(address);
#if ARDUINO >= 100
    Wire.write((byte)theRegister);
#else
    Wire.send((byte)theRegister);
#endif
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0)
    {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  }
  else
  {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available())
  {
    Firmata.sendString("I2C Read Error: Too many bytes received");
  }
  else if (numBytes > Wire.available())
  {
    Firmata.sendString("I2C Read Error: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++)
  {
#if ARDUINO >= 100
    i2cRxData[2 + i] = Wire.read();
#else
    i2cRxData[2 + i] = Wire.receive();
#endif
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue)
  {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C)
  {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != SERVO)
  {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached())
    {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin))
  {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin))
  {
    if (mode == INPUT || mode == FIRMATA_INPUT_PULLUP)
    {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    }
    else
    {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode)
  {
  case ANALOG:
    if (IS_PIN_ANALOG(pin))
    {
      if (IS_PIN_DIGITAL(pin))
      {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      }
      pinConfig[pin] = ANALOG;
    }
    break;
  case INPUT:
    if (IS_PIN_DIGITAL(pin))
    {
      pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      pinConfig[pin] = INPUT;
    }
    break;
  case OUTPUT:
    if (IS_PIN_DIGITAL(pin))
    {
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
      pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      pinConfig[pin] = OUTPUT;
    }
    break;
  case PWM:
    if (IS_PIN_PWM(pin))
    {
      pinMode(PIN_TO_PWM(pin), OUTPUT);
      analogWrite(PIN_TO_PWM(pin), 0);
      pinConfig[pin] = PWM;
    }
    break;
  case SERVO:
    if (IS_PIN_DIGITAL(pin))
    {
      pinConfig[pin] = SERVO;
      if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached())
      {
        // pass -1 for min and max pulse values to use default values set
        // by Servo library
        attachServo(pin, -1, -1);
      }
    }
    break;
  case I2C:
    if (IS_PIN_I2C(pin))
    {
      // mark the pin as i2c
      // the user must call I2C_CONFIG to enable I2C for a device
      pinConfig[pin] = I2C;
    }
    break;
  case FIRMATA_INPUT_PULLUP:
    if (IS_PIN_DIGITAL(pin)) {
      pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP); // disable output driver
      //digitalWrite(PIN_TO_DIGITAL(pin), HIGH); // enable internal pull-ups
      pinConfig[pin] = FIRMATA_INPUT_PULLUP;
    }
    break;
  case STEPPER:
    if (IS_PIN_DIGITAL(pin)) {
      pinConfig[pin] = STEPPER;
    }
    break;
  case ENCODER:
    //if (IS_PIN_INTERRUPT(pin)) 
    //{
    pinConfig[pin] = ENCODER;
    //}
    break;
  case ONEWIRE:
    if (IS_PIN_DIGITAL(pin)) 
    {
      oneWireConfig(pin,ONEWIRE_POWER);
      pinConfig[pin] = ONEWIRE;
    }
    break;
  default:
    Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS)
  {
    switch (pinConfig[pin])
    {
    case SERVO:
      if (IS_PIN_DIGITAL(pin))
        servos[servoPinMap[pin]].write(value);
      pinState[pin] = value;
      break;
    case PWM:
      if (IS_PIN_PWM(pin))
        analogWrite(PIN_TO_PWM(pin), value);
      pinState[pin] = value;
      break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS)
  {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++)
    {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin))
      {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT)
        {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS)
  {
    if (value == 0)
    {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    }
    else
    {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // Send pin value immediately. This is helpful when connected via
      // ethernet, wi-fi or bluetooth so pin states can be known upon
      // reconnecting.
      Firmata.sendAnalog(analogPin, analogRead(analogPin));
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS)
  {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command)
  {
  case I2C_REQUEST:
    mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
    if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK)
    {
      Firmata.sendString("10-bit addressing not supported");
      return;
    }
    else
    {
      slaveAddress = argv[0];
    }

    switch (mode)
    {
    case I2C_WRITE:
      Wire.beginTransmission(slaveAddress);
      for (byte i = 2; i < argc; i += 2)
      {
        data = argv[i] + (argv[i + 1] << 7);
#if ARDUINO >= 100
        Wire.write(data);
#else
        Wire.send(data);
#endif
      }
      Wire.endTransmission();
      delayMicroseconds(70);
      break;
    case I2C_READ:
      if (argc == 6)
      {
        // a slave register is specified
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7);  // bytes to read
      }
      else
      {
        // a slave register is NOT specified
        slaveRegister = REGISTER_NOT_SPECIFIED;
        data = argv[2] + (argv[3] << 7);  // bytes to read
      }
      readAndReportData(slaveAddress, (int)slaveRegister, data);
      break;
    case I2C_READ_CONTINUOUSLY:
      if ((queryIndex + 1) >= MAX_QUERIES)
      {
        // too many queries, just ignore
        Firmata.sendString("too many queries");
        break;
      }
      if (argc == 6)
      {
        // a slave register is specified
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7);  // bytes to read
      }
      else
      {
        // a slave register is NOT specified
        slaveRegister = (int)REGISTER_NOT_SPECIFIED;
        data = argv[2] + (argv[3] << 7);  // bytes to read
      }
      queryIndex++;
      query[queryIndex].addr = slaveAddress;
      query[queryIndex].reg = slaveRegister;
      query[queryIndex].bytes = data;
      break;
    case I2C_STOP_READING:
      byte queryIndexToSkip;
      // if read continuous mode is enabled for only 1 i2c device, disable
      // read continuous reporting for that device
      if (queryIndex <= 0)
      {
        queryIndex = -1;
      }
      else
      {
        // if read continuous mode is enabled for multiple devices,
        // determine which device to stop reading and remove it's data from
        // the array, shifiting other array data to fill the space
        for (byte i = 0; i < queryIndex + 1; i++)
        {
          if (query[i].addr == slaveAddress)
          {
            queryIndexToSkip = i;
            break;
          }
        }

        for (byte i = queryIndexToSkip; i < queryIndex + 1; i++)
        {
          if (i < MAX_QUERIES)
          {
            query[i].addr = query[i + 1].addr;
            query[i].reg = query[i + 1].reg;
            query[i].bytes = query[i + 1].bytes;
          }
        }
        queryIndex--;
      }
      break;
    default:
      break;
    }
    break;
  case I2C_CONFIG:
    delayTime = (argv[0] + (argv[1] << 7));

    if (delayTime > 0)
    {
      i2cReadDelayTime = delayTime;
    }

    if (!isI2CEnabled)
    {
      enableI2CPins();
    }

    break;
  case SERVO_CONFIG:
    if (argc > 4)
    {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte pin = argv[0];
      int minPulse = argv[1] + (argv[2] << 7);
      int maxPulse = argv[3] + (argv[4] << 7);

      if (IS_PIN_DIGITAL(pin))
      {
        if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached())
        {
          detachServo(pin);
        }
        attachServo(pin, minPulse, maxPulse);
        setPinModeCallback(pin, SERVO);
      }
    }
    break;

  case STEPPER_DATA:
    byte stepCommand, deviceNum, directionPin, stepPin, stepDirection;
    byte interface, interfaceType;
    byte motorPin3, motorPin4, limitSwitch1, limitSwitch2;
    unsigned int stepsPerRev;
    long numSteps;
    int stepSpeed;
    int accel;
    int decel;
    boolean l1usePullup, l2usePullup;

    stepCommand = argv[0];
    deviceNum = argv[1];

    if (deviceNum < MAX_STEPPERS)
    {
      if (stepCommand == STEPPER_CONFIG)
      {
        interface = argv[2]; // upper 4 bits are the stepDelay, lower 4 bits are the interface type
        interfaceType = interface & 0x0F; // the interface type is specified by the lower 4 bits
        stepsPerRev = (argv[3] + (argv[4] << 7));

        directionPin = argv[5]; // or motorPin1 for TWO_WIRE or FOUR_WIRE interface
        stepPin = argv[6]; // // or motorPin2 for TWO_WIRE or FOUR_WIRE interface
        setPinModeCallback(directionPin, STEPPER);
        setPinModeCallback(stepPin, STEPPER);

        if (!stepper[deviceNum])
        {
          numSteppers++; // assumes steppers are added in order 0 -> 5
        }

        if (interfaceType == FirmataStepper::DRIVER || interfaceType == FirmataStepper::TWO_WIRE)
        {
          limitSwitch1= argv[7];
          limitSwitch2 = argv[8];
          l1usePullup = argv[9];
          l2usePullup = argv[10];
          stepper[deviceNum] = new FirmataStepper(interface, stepsPerRev, directionPin, stepPin, limitSwitch1, limitSwitch2, l1usePullup, l2usePullup);
        }
        else if (interfaceType == FirmataStepper::FOUR_WIRE)
        {
          motorPin3 = argv[7];
          motorPin4 = argv[8];
          limitSwitch1= argv[9];
          limitSwitch2 = argv[10];
          l1usePullup = argv[11];
          l2usePullup = argv[12];
          setPinModeCallback(motorPin3, STEPPER);
          setPinModeCallback(motorPin4, STEPPER);
          stepper[deviceNum] = new FirmataStepper(interface, stepsPerRev, directionPin, stepPin, motorPin3, motorPin4, limitSwitch1, limitSwitch2, l1usePullup, l2usePullup);
        }
      }
      else if (stepCommand == STEPPER_MOVE)
      {
        stepDirection = argv[2];
        numSteps = (long)argv[3] | ((long)argv[4] << 7) | ((long)argv[5] << 14);

        if (stepDirection == 0)
        {
          numSteps *= -1;
        }
        if (stepper[deviceNum])
        {
          if (argc < 8)
          {
            stepper[deviceNum]->setStepsToMove(numSteps);
          }
          if (argc >= 8 && argc < 12)
          {
            stepSpeed = (argv[6] + (argv[7] << 7));
            // num steps, speed (0.01*rad/sec)
            stepper[deviceNum]->setStepsToMove(numSteps, stepSpeed);
          }
          else if (argc == 12)
          {
            stepSpeed = (argv[6] + (argv[7] << 7));
            accel = (argv[8] + (argv[9] << 7));
            decel = (argv[10] + (argv[11] << 7));
            // num steps, speed (0.01*rad/sec), accel (0.01*rad/sec^2), decel (0.01*rad/sec^2)
            stepper[deviceNum]->setStepsToMove(numSteps, stepSpeed, accel, decel);
          }
        }
      } 
      else if (stepCommand == STEPPER_GET_POSITION)
      {
        long curPosition = stepper[deviceNum]->getPosition();
        Serial.write(START_SYSEX);
        Serial.write(STEPPER_DATA);
        Serial.write(STEPPER_GET_POSITION);
        Serial.write(deviceNum);
        long absValue = abs(curPosition);
        Serial.write((byte)absValue & 0x7F);
        Serial.write((byte)(absValue >> 7) & 0x7F);
        Serial.write((byte)(absValue >> 14) & 0x7F);
        Serial.write((byte)(absValue >> 21) & 0x7F);
        //this is a signed value we need the bit flag
        Serial.write(curPosition >= 0? 0x01 : 0x00);
        Serial.write(END_SYSEX);

        char str[64];
        sprintf(str, "%d", curPosition);
        Firmata.sendString(str);

      }
      else if (stepCommand == STEPPER_GET_DISTANCE_TO)
      {
        long distTo = stepper[deviceNum]->getDistanceTo();
        Serial.write(START_SYSEX);
        Serial.write(STEPPER_DATA);
        Serial.write(STEPPER_GET_DISTANCE_TO);
        Serial.write(deviceNum);
        long absValue = abs(distTo);
        Serial.write((byte)absValue & 0x7F);
        Serial.write((byte)(absValue >> 7) & 0x7F);
        Serial.write((byte)(absValue >> 14) & 0x7F);
        Serial.write((byte)(absValue >> 21) & 0x7F);
        //this is a signed value we need the bit flag
        Serial.write(distTo >= 0? 0x01 : 0x00);
        Serial.write(END_SYSEX);

        char str[64];
        sprintf(str, "%d", distTo);
        Firmata.sendString(str);

      }
      else if (stepCommand == STEPPER_SET_SPEED)
      {
        int newSpeed = (argv[2] + (argv[3] << 7));
        stepper[deviceNum]->setSpeed(newSpeed);
      }
      else if (stepCommand == STEPPER_SET_ACCEL)
      {
        int newAccel = (argv[2] + (argv[3] << 7));
        stepper[deviceNum]->setAcceleration(newAccel);
      }
      else if (stepCommand == STEPPER_SET_DECEL)
      {
        int newDecel = (argv[2] + (argv[3] << 7));
        stepper[deviceNum]->setDeceleration(newDecel);
      }
    }
    break;

  case ENCODER_DATA:
    byte encoderCommand, encoderNum, pinA, pinB, enableReports; 

    encoderCommand= argv[0];

    if (encoderCommand == ENCODER_ATTACH) 
    {
      encoderNum = argv[1];
      pinA = argv[2];
      pinB = argv[3];
      if (pinConfig[pinA]!=IGNORE && pinConfig[pinB]!=IGNORE)
      {
        attachEncoder(encoderNum, pinA, pinB);
      }      
    }

    if (encoderCommand == ENCODER_REPORT_POSITION)
    {
      encoderNum = argv[1];
      reportEncoderPosition(encoderNum);
    }

    if (encoderCommand == ENCODER_REPORT_POSITIONS)
    {
      reportEncoderPositions();
    }

    if (encoderCommand == ENCODER_RESET_POSITION)
    {
      encoderNum = argv[1];
      resetEncoderPosition(encoderNum);
    }
    if (encoderCommand == ENCODER_REPORT_AUTO)
    {
      reportEncoders = argv[1];
    }

    if (encoderCommand == ENCODER_DETACH)
    {
      encoderNum = argv[1];
      detachEncoder(encoderNum);
    }
    break;
    if (command == ONEWIRE_DATA) {
      if (argc>1) {
        byte subcommand = argv[0];
        byte pin = argv[1];
        ow_device_info *info = &pinOneWire[pin];
        OneWire *device = info->device;
        if (device || subcommand == ONEWIRE_CONFIG_REQUEST) {
          switch(subcommand) {
          case ONEWIRE_SEARCH_REQUEST:
          case ONEWIRE_SEARCH_ALARMS_REQUEST:
            {
              device->reset_search();
              Serial.write(START_SYSEX);
              Serial.write(ONEWIRE_DATA);
              boolean isAlarmSearch = (subcommand == ONEWIRE_SEARCH_ALARMS_REQUEST);
              Serial.write(isAlarmSearch ? (byte)ONEWIRE_SEARCH_ALARMS_REPLY : (byte)ONEWIRE_SEARCH_REPLY);
              Serial.write(pin);
              Encoder7Bit.startBinaryWrite();
              byte addrArray[8];
              while (isAlarmSearch ? device->search_alarms(addrArray) : device->search(addrArray)) {
                for (int i=0;i<8;i++) {
                  Encoder7Bit.writeBinary(addrArray[i]);
                }
              }
              Encoder7Bit.endBinaryWrite();
              Serial.write(END_SYSEX);
              break;
            }
          case ONEWIRE_CONFIG_REQUEST:
            {
              if (argc==3 && pinConfig[pin]!=IGNORE) {
                pinConfig[pin] = ONEWIRE;
                oneWireConfig(pin, argv[2]); // this calls oneWireConfig again, this time setting the correct config (which doesn't cause harm though)
              } 
              break;
            }
          default:
            {
              if (subcommand & ONEWIRE_RESET_REQUEST_BIT) {
                device->reset();
                for (int i=0;i<8;i++) {
                  info->addr[i]=0x0;
                }
              }
              if (subcommand & ONEWIRE_SKIP_REQUEST_BIT) {
                device->skip();
                for (byte i=0;i<8;i++) {
                  info->addr[i]=0x0;
                }
              }
              if (subcommand & ONEWIRE_WITHDATA_REQUEST_BITS) {
                int numBytes=num7BitOutbytes(argc-2);
                int numReadBytes=0;
                int correlationId;
                argv+=2;
                Encoder7Bit.readBinary(numBytes,argv,argv); //decode inplace

                if (subcommand & ONEWIRE_SELECT_REQUEST_BIT) {
                  if (numBytes<8) break;
                  device->select(argv);
                  for (int i=0;i<8;i++) {
                    info->addr[i]=argv[i];
                  }
                  argv+=8;
                  numBytes-=8;
                }

                if (subcommand & ONEWIRE_READ_REQUEST_BIT) {
                  if (numBytes<4) break;
                  numReadBytes = *((int*)argv);
                  argv+=2;
                  correlationId = *((int*)argv);
                  argv+=2;
                  numBytes-=4;
                }

                if (subcommand & ONEWIRE_WRITE_REQUEST_BIT) {
                  for (int i=0;i<numBytes;i++) {
                    info->device->write(argv[i],info->power);
                  }
                }

                if (numReadBytes>0) {
                  Serial.write(START_SYSEX);
                  Serial.write(ONEWIRE_DATA);
                  Serial.write(ONEWIRE_READ_REPLY);
                  Serial.write(pin);
                  Encoder7Bit.startBinaryWrite();
                  Encoder7Bit.writeBinary(correlationId&0xFF);
                  Encoder7Bit.writeBinary((correlationId>>8)&0xFF);
                  for (int i=0;i<numReadBytes;i++) {
                    Encoder7Bit.writeBinary(device->read());
                  }
                  Encoder7Bit.endBinaryWrite();
                  Serial.write(END_SYSEX);
                }
              }
            }
          }
        }
      }
    }
  case SAMPLING_INTERVAL:
    if (argc > 1)
    {
      samplingInterval = argv[0] + (argv[1] << 7);
      if (samplingInterval < MINIMUM_SAMPLING_INTERVAL)
      {
        samplingInterval = MINIMUM_SAMPLING_INTERVAL;
      }
    }
    else
    {
      //Firmata.sendString("Not enough data");
    }
    break;
  case EXTENDED_ANALOG:
    if (argc > 1)
    {
      int val = argv[1];
      if (argc > 2) val |= (argv[2] << 7);
      if (argc > 3) val |= (argv[3] << 14);
      analogWriteCallback(argv[0], val);
    }
    break;
  case CAPABILITY_QUERY:
    Serial.write(START_SYSEX);
    Serial.write(CAPABILITY_RESPONSE);
    for (byte pin = 0; pin < TOTAL_PINS; pin++)
    {
      if (IS_PIN_DIGITAL(pin))
      {
        Serial.write((byte)INPUT);
        Serial.write(1);
        Serial.write((byte)OUTPUT);
        Serial.write(1);
      }
      if (IS_PIN_ANALOG(pin))
      {
        Serial.write(ANALOG);
        Serial.write(10);
      }
      if (IS_PIN_PWM(pin))
      {
        Serial.write(PWM);
        Serial.write(8);
      }
      if (IS_PIN_DIGITAL(pin))
      {
        Serial.write(SERVO);
        Serial.write(14);
      }
      if (IS_PIN_I2C(pin))
      {
        Serial.write(I2C);
        Serial.write(1);  // to do: determine appropriate value
      }
      if (IS_PIN_DIGITAL(pin))
      {
        Serial.write(STEPPER);
        Serial.write(21); //21 bits used for number of steps
      }
      if (IS_PIN_DIGITAL(pin))
      {//(IS_PIN_INTERRUPT(pin)) {
        Serial.write(ENCODER);
        Serial.write(28); //28 bits used for absolute position
      }
      if (IS_PIN_DIGITAL(pin)) {
        Serial.write(ONEWIRE);
        Serial.write(1);
      }
      Serial.write(127);
    }
    Serial.write(END_SYSEX);
    break;
  case PIN_STATE_QUERY:
    if (argc > 0)
    {
      byte pin = argv[0];
      Serial.write(START_SYSEX);
      Serial.write(PIN_STATE_RESPONSE);
      Serial.write(pin);
      if (pin < TOTAL_PINS)
      {
        Serial.write((byte)pinConfig[pin]);
        Serial.write((byte)pinState[pin] & 0x7F);
        if (pinState[pin] & 0xFF80) Serial.write((byte)(pinState[pin] >> 7) & 0x7F);
        if (pinState[pin] & 0xC000) Serial.write((byte)(pinState[pin] >> 14) & 0x7F);
      }
      Serial.write(END_SYSEX);
    }
    break;
  case ANALOG_MAPPING_QUERY:
    Serial.write(START_SYSEX);
    Serial.write(ANALOG_MAPPING_RESPONSE);
    for (byte pin = 0; pin < TOTAL_PINS; pin++)
    {
      Serial.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
    }
    Serial.write(END_SYSEX);
    break;
  default:
    Firmata.sendString("Unknown sysex command");
    break;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++)
  {
    if (IS_PIN_I2C(i))
    {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    }
  }

  isI2CEnabled = true;

  // is there enough time before the first I2C request to call this here?
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins()
{
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
  // uncomment the following if or when the end() method is added to Wire library
  // Wire.end();
}

void resetEncoderPosition(byte encoderNum)
{
  if (isEncoderAttached(encoderNum))
  {
    encoders[encoderNum].write(0);
  }
}

// Report specific encoder position using midi protocol
void reportEncoderPosition(byte encoder)
{
  if (isEncoderAttached(encoder))
  {
    Serial.write(START_SYSEX);
    Serial.write(ENCODER_DATA);
    long absValue = abs(positions[encoder]);
    byte direction = positions[encoder] >= 0 ? 0x00 : 0x01;
    Serial.write((direction << 6) | (encoder));
    Serial.write((byte)absValue & 0x7F);
    Serial.write((byte)(absValue >> 7) & 0x7F);
    Serial.write((byte)(absValue >> 14) & 0x7F);
    Serial.write((byte)(absValue >> 21) & 0x7F);
    Serial.write(END_SYSEX);
  }
}

// Report all attached encoders positions (one message for all encoders) 
void reportEncoderPositions()
{
  bool report = false;
  for (uint8_t encoderNum=0; encoderNum < MAX_ENCODERS; encoderNum++)
  {
    if (isEncoderAttached(encoderNum))
    {
      //int32_t encPosition = encoders[encoderNum].read();
      /*char str[64];
       sprintf(str, "%d", encPosition);
       Firmata.sendString(str);*/
      if (positions[encoderNum] != prevPositions[encoderNum])
      {
        if (!report)
        {
          Firmata.write(START_SYSEX);
          Firmata.write(ENCODER_DATA);
          report = true;
        }

        prevPositions[encoderNum] = positions[encoderNum];
        long absValue = abs(positions[encoderNum]);
        byte direction = positions[encoderNum] >= 0 ? 0x00 : 0x01;
        Serial.write((direction << 6) | (encoderNum));
        Serial.write((byte)absValue & 0x7F);
        Serial.write((byte)(absValue >> 7) & 0x7F);
        Serial.write((byte)(absValue >> 14) & 0x7F);
        Serial.write((byte)(absValue >> 21) & 0x7F);
      }
    }
  }
  if (report)
  {
    Firmata.write(END_SYSEX);
  }
}

boolean isEncoderAttached(byte encoderNum){
  return (encoderNum < MAX_ENCODERS && encoderNum < numAttachedEncoders/*&& encoders[encoderNum]*/);
}


/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default
  if (isI2CEnabled)
  {
    disableI2CPins();
  }
  for (byte i = 0; i < TOTAL_PORTS; i++)
  {
    reportPINs[i] = false;      // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }
  // pins with analog capability default to analog input
  // otherwise, pins default to digital output
  for (byte i = 0; i < TOTAL_PINS; i++)
  {
    if (IS_PIN_ANALOG(i))
    {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    }
    else
    {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  for (byte i = 0; i < MAX_STEPPERS; i++)
  {
    if (stepper[i])
    {
      free(stepper[i]);
      stepper[i] = 0;
    }
  }
  numSteppers = 0;
  detachedServoCount = 0;
  servoCount = 0;

  byte encoder;
  for(encoder=0; encoder<MAX_ENCODERS; encoder++) 
  {
    detachEncoder(encoder);
  }
  reportEncoders = 0x00;


  for (int i=0;i<TOTAL_PINS;i++) {
    if (pinOneWire[i].device) {
      free(pinOneWire[i].device);
      pinOneWire[i].device=NULL;
    }
    for (int j=0;j<8;j++) {
      pinOneWire[i].addr[j]=0;
    }
    pinOneWire[i].power=false;
  }
  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
   but it will be needed when/if we support EEPROM stored config
   for (byte i=0; i < TOTAL_PORTS; i++) {
   outputPort(i, readPort(i, portConfigInputs[i]), true);
   }
   */
}

void setup()
{
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  Firmata.begin(57600);
  systemResetCallback();  // reset to default config
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* SERIALREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  /* SEND FTDI WRITE BUFFER - make sure that the FTDI buffer doesn't go over
   * 60 bytes. use a timer to sending an event character every 4 ms to
   * trigger the buffer to dump. */

  // if one or more stepper motors are used, update their position
  if (numSteppers > 0)
  {
    for (int i = 0; i < MAX_STEPPERS; i++)
    {
      if (stepper[i])
      {
        bool done = stepper[i]->update();
        // send command to client application when stepping is complete
        if (done)
        {
          Serial.write(START_SYSEX);
          Serial.write(STEPPER_DATA);
          Serial.write(STEPPER_DONE);
          Serial.write(i & 0x7F);
          Serial.write(END_SYSEX);
        }
      }
    }
  }

  //the delay in the reporting interval causes encoders to report incorrectly
  //need to refresh them faster
  for(byte i = 0; i < numAttachedEncoders; i++)
  {
    int32_t encPosition = encoders[i].read();
    if (positions[i] != encPosition)
      positions[i] = encPosition;
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval)
  {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++)
    {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG)
      {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin))
        {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1)
    {
      for (byte i = 0; i < queryIndex + 1; i++)
      {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
    if (reportEncoders)
    {
      reportEncoderPositions();
    }
  }
}













