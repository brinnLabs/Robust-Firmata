/*
  Firmata.h - Firmata library
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef Firmata_h
#define Firmata_h

#include "Boards.h"  /* Hardware Abstraction Layer + Wiring/Arduino */

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.  This number can be queried so that host
 * software can test whether it will be compatible with the currently
 * installed firmware. */
#define FIRMATA_MAJOR_VERSION   2 // for non-compatible changes
#define FIRMATA_MINOR_VERSION   3 // for backwards compatible changes
#define FIRMATA_BUGFIX_VERSION  6 // for bugfix releases

#define MAX_DATA_BYTES 32 // max number of data bytes in non-Sysex messages

// message command bytes (128-255/0x80-0xFF)
#define DIGITAL_MESSAGE         0x90 // send data for a digital pin
#define ANALOG_MESSAGE          0xE0 // send data for an analog pin (or PWM)
#define REPORT_ANALOG           0xC0 // enable analog input by pin #
#define REPORT_DIGITAL          0xD0 // enable digital input by port pair
//
#define SET_PIN_MODE            0xF4 // set a pin to INPUT/OUTPUT/PWM/etc
//
#define REPORT_VERSION          0xF9 // report protocol version
#define SYSTEM_RESET            0xFF // reset from MIDI
//
#define START_SYSEX             0xF0 // start a MIDI Sysex message
#define END_SYSEX               0xF7 // end a MIDI Sysex message

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */
#define SERVO_CONFIG            0x70 // set max angle, minPulse, maxPulse, freq
#define STRING_DATA             0x71 // a string message with 14-bits per char
#define SHIFT_DATA              0x75 // a bitstream to/from a shift register
#define I2C_REQUEST             0x76 // send an I2C read/write request
#define I2C_REPLY               0x77 // a reply to an I2C read request
#define I2C_CONFIG              0x78 // config I2C settings such as delay times and power pins
#define EXTENDED_ANALOG         0x6F // analog write (PWM, Servo, etc) to any pin
#define PIN_STATE_QUERY         0x6D // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE      0x6E // reply with pin's current mode and value
#define CAPABILITY_QUERY        0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE     0x6C // reply with supported modes and resolution
#define ANALOG_MAPPING_QUERY    0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE 0x6A // reply with mapping info
#define REPORT_FIRMWARE         0x79 // report name and version of the firmware
#define SAMPLING_INTERVAL       0x7A // set the poll rate of the main loop
#define SYSEX_NON_REALTIME      0x7E // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME          0x7F // MIDI Reserved for realtime messages
// these are DEPRECATED to make the naming more consistent
#define FIRMATA_STRING          0x71 // same as STRING_DATA
#define SYSEX_I2C_REQUEST       0x76 // same as I2C_REQUEST
#define SYSEX_I2C_REPLY         0x77 // same as I2C_REPLY
#define SYSEX_SAMPLING_INTERVAL 0x7A // same as SAMPLING_INTERVAL

#define MAX_STEPPERS    		6     // arbitrary value... may need to adjust
#define STEPPER_DATA    		0x72  // move this to Firmata.h
#define STEPPER_CONFIG  		0
#define STEPPER_STEP    		1

#define I2C_WRITE 					B00000000
#define I2C_READ 					B00001000
#define I2C_READ_CONTINUOUSLY 		B00010000
#define I2C_STOP_READING 			B00011000
#define I2C_READ_WRITE_MODE_MASK 	B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
  
#define MAX_QUERIES 8 // max number of i2c devices in read continuous mode
#define MINIMUM_SAMPLING_INTERVAL 10
#define REGISTER_NOT_SPECIFIED -1

// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h
#define ANALOG                  0x02 // analog pin in analogInput mode
#define PWM                     0x03 // digital pin in PWM output mode
#define SERVO                   0x04 // digital pin in Servo output mode
#define SHIFT                   0x05 // shiftIn/shiftOut mode
#define I2C                     0x06 // pin included in I2C setup
#define FIRMATA_INPUT_PULLUP	0x07 // enable pullup resistors
#define STEPPER         		0x08  // pin configured for stepper motor
#define TOTAL_PIN_MODES         9

extern "C" {
// callback function types
    typedef void (*callbackFunction)(byte, int);
    typedef void (*systemResetCallbackFunction)(void);
    typedef void (*stringCallbackFunction)(char*);
    typedef void (*sysexCallbackFunction)(byte command, byte argc, byte*argv);
}


// TODO make it a subclass of a generic Serial/Stream base class
class FirmataClass
{
public:
	FirmataClass(Stream &s);
/* Arduino constructors */
    void begin();
    void begin(long);
    void begin(Stream &s);
/* querying functions */
	void printVersion(void);
    void blinkVersion(void);
    void printFirmwareVersion(void);
  //void setFirmwareVersion(byte major, byte minor);  // see macro below
    void setFirmwareNameAndVersion(const char *name, byte major, byte minor);
/* serial receive handling */
    int available(void);
    void processInput(void);
/* serial send handling */
	void sendAnalog(byte pin, int value);
	void sendDigital(byte pin, int value); // TODO implement this
	void sendDigitalPort(byte portNumber, int portData);
    void sendString(const char* string);
    void sendString(byte command, const char* string);
	void sendSysex(byte command, byte bytec, byte* bytev);
/* attach & detach callback functions to messages */
    void attach(byte command, callbackFunction newFunction);
    void attach(byte command, systemResetCallbackFunction newFunction);
    void attach(byte command, stringCallbackFunction newFunction);
    void attach(byte command, sysexCallbackFunction newFunction);
    void detach(byte command);

private:
    Stream &FirmataSerial;
/* firmware name and version */
    byte firmwareVersionCount;
    byte *firmwareVersionVector;
/* input message handling */
    byte waitForData; // this flag says the next serial input will be data
    byte executeMultiByteCommand; // execute this after getting multi-byte data
    byte multiByteChannel; // channel data for multiByteCommands
    byte storedInputData[MAX_DATA_BYTES]; // multi-byte data
/* sysex */
    boolean parsingSysex;
    int sysexBytesRead;
/* callback functions */
    callbackFunction currentAnalogCallback;
    callbackFunction currentDigitalCallback;
    callbackFunction currentReportAnalogCallback;
    callbackFunction currentReportDigitalCallback;
    callbackFunction currentPinModeCallback;
    systemResetCallbackFunction currentSystemResetCallback;
    stringCallbackFunction currentStringCallback;
    sysexCallbackFunction currentSysexCallback;

/* private methods ------------------------------ */
    void processSysexMessage(void);
	void systemReset(void);
    void pin13strobe(int count, int onInterval, int offInterval);
    void sendValueAsTwo7bitBytes(int value);
    void startSysex(void);
    void endSysex(void);
};

extern FirmataClass Firmata;

/*==============================================================================
 * MACROS
 *============================================================================*/

/* shortcut for setFirmwareNameAndVersion() that uses __FILE__ to set the
 * firmware name.  It needs to be a macro so that __FILE__ is included in the
 * firmware source file rather than the library source file.
 */
#define setFirmwareVersion(x, y)   setFirmwareNameAndVersion(__FILE__, x, y)

#endif /* Firmata_h */

/*
  FirmataStepper is a simple non-blocking stepper motor library
  for 2 and 4 wire bipolar and unipolar stepper motor drive circuits
  as well as EasyDriver (http://schmalzhaus.com/EasyDriver/) and 
  other step + direction drive circuits.
  FirmataStepper (0.1) by Jeff Hoefs
  
  EasyDriver support based on modifications by Chris Coleman
  Acceleration / Deceleration algorithms and code based on:
  app note: http://www.atmel.com/dyn/resources/prod_documents/doc8017.pdf
  source code: http://www.atmel.com/dyn/resources/prod_documents/AVR446.zip
  
  stepMotor function based on Stepper.cpp Stepper library for
  Wiring/Arduino created by Tom Igoe, Sebastian Gassner
  David Mellis and Noah Shibley.
  Relevant notes from Stepper.cpp:
  When wiring multiple stepper motors to a microcontroller,
  you quickly run out of output pins, with each motor requiring 4 connections. 
  By making use of the fact that at any time two of the four motor
  coils are the inverse  of the other two, the number of
  control connections can be reduced from 4 to 2. 
  A slightly modified circuit around a Darlington transistor array or an L293 H-bridge
  connects to only 2 microcontroler pins, inverts the signals received,
  and delivers the 4 (2 plus 2 inverted ones) output signals required
  for driving a stepper motor.
  The sequence of control signals for 4 control wires is as follows:
  Step C0 C1 C2 C3
     1  1  0  1  0
     2  0  1  1  0
     3  0  1  0  1
     4  1  0  0  1
  The sequence of controls signals for 2 control wires is as follows
  (columns C1 and C2 from above):
  Step C0 C1
     1  0  1
     2  1  1
     3  1  0
     4  0  0
  The circuits can be found at 
  http://www.arduino.cc/en/Tutorial/Stepper
*/

// ensure this library description is only included once
#ifndef FirmataStepper_h
#define FirmataStepper_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define PI_2 2*3.14159
#define T1_FREQ 1000000L // provides the most accurate step delay values
#define T1_FREQ_148 ((long)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676

// library interface description
class FirmataStepper {
  public:
    FirmataStepper(byte interface = FirmataStepper::DRIVER, 
                    int steps_per_rev = 200, 
                    byte pin1 = 2, 
                    byte pin2 = 3, 
                    byte pin3 = 3, 
                    byte pin4 = 4);

    enum Interface {
      DRIVER = 1,
      TWO_WIRE = 2,
      FOUR_WIRE = 4
    };

    enum RunState {
      STOP = 0,
      ACCEL = 1,
      DECEL = 2,
      RUN = 3
    };

    enum Direction {
      CCW = 0,
      CW = 1
    };

    void setStepsToMove(long steps_to_move, int speed, int accel=0, int decel=0);

    // update the stepper position
		bool update();

    byte version(void);

  private:
    void stepMotor(byte step_num, byte direction);
    void updateStepPosition();
    bool running;
    byte interface;     // Type of interface: DRIVER, TWO_WIRE or FOUR_WIRE
    byte direction;        // Direction of rotation
    unsigned long step_delay;    // delay between steps, in microseconds
    int steps_per_rev;      // number of steps to make one revolution
    long step_number;        // which step the motor is on
    long steps_to_move;   // total number of teps to move

    byte run_state;
    int accel_count;
    long min_delay;
    long decel_start;
    int decel_val;

    long lastAccelDelay;
    unsigned long stepCount;
    unsigned int rest;    

    float alpha;  // PI * 2 / steps_per_rev
    long at_x100;  // alpha * T1_FREQ * 100
    long ax20000;  // alph a* 20000
    float alpha_x2;  // alpha * 2
    
    // motor pin numbers:
    byte dir_pin;
    byte step_pin;
    byte motor_pin_1;
    byte motor_pin_2;
    byte motor_pin_3;
    byte motor_pin_4;
    
    unsigned long last_step_time; // time stamp in microseconds of when the last step was taken
};

#endif
