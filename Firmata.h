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
#include "Utility/Encoder.h"
#include "Utility/OneWire.h"
#include "Utility/AccelStepper.h"

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.  This number can be queried so that host
 * software can test whether it will be compatible with the currently
 * installed firmware. */
#define FIRMATA_MAJOR_VERSION   2 // for non-compatible changes
#define FIRMATA_MINOR_VERSION   4 // for backwards compatible changes
#define FIRMATA_BUGFIX_VERSION  0 // for bugfix releases

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
#define ENCODER_DATA            0x61 // reply with encoders current positions
#define SERVO_CONFIG            0x70 // set max angle, minPulse, maxPulse, freq
#define STRING_DATA             0x71 // a string message with 14-bits per char
#define STEPPER_DATA            0x72 // control a stepper motor
#define ONEWIRE_DATA            0x73 // send an OneWire read/write/reset/select/skip/search request
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
#define SCHEDULER_DATA          0x7B // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
#define SYSEX_NON_REALTIME      0x7E // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME          0x7F // MIDI Reserved for realtime messages
// these are DEPRECATED to make the naming more consistent
#define FIRMATA_STRING          0x71 // same as STRING_DATA
#define SYSEX_I2C_REQUEST       0x76 // same as I2C_REQUEST
#define SYSEX_I2C_REPLY         0x77 // same as I2C_REPLY
#define SYSEX_SAMPLING_INTERVAL 0x7A // same as SAMPLING_INTERVAL

#define MAX_ENCODERS                5 // arbitrary value, may need to adjust
#define ENCODER_ATTACH              0x00
#define ENCODER_REPORT_POSITION     0x01
#define ENCODER_REPORT_POSITIONS    0x02
#define ENCODER_RESET_POSITION      0x03
#define ENCODER_REPORT_AUTO         0x04
#define ENCODER_DETACH              0x05

#define MAX_STEPPERS    			6     // arbitrary value... may need to adjust
#define STEPPER_DATA    			0x72  // move this to Firmata.h
#define STEPPER_CONFIG  			0
#define STEPPER_MOVE    			1
#define STEPPER_DONE				10
#define STEPPER_GET_POSITION		2
#define STEPPER_GET_DISTANCE_TO		3
#define STEPPER_GET_SPEED			4
#define STEPPER_SET_POSITION		5
#define STEPPER_SET_MAX_SPEED		6
#define STEPPER_SET_SPEED			7
#define STEPPER_SET_ACCEL			8
#define STEPPER_SET_DECEL			9



#define I2C_WRITE 					B00000000
#define I2C_READ 					B00001000
#define I2C_READ_CONTINUOUSLY 		B00010000
#define I2C_STOP_READING 			B00011000
#define I2C_READ_WRITE_MODE_MASK 	B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

//subcommands:
#define ONEWIRE_SEARCH_REQUEST 0x40
#define ONEWIRE_CONFIG_REQUEST 0x41
#define ONEWIRE_SEARCH_REPLY 0x42
#define ONEWIRE_READ_REPLY 0x43
#define ONEWIRE_SEARCH_ALARMS_REQUEST 0x44
#define ONEWIRE_SEARCH_ALARMS_REPLY 0x45

#define ONEWIRE_RESET_REQUEST_BIT 0x01
#define ONEWIRE_SKIP_REQUEST_BIT 0x02
#define ONEWIRE_SELECT_REQUEST_BIT 0x04
#define ONEWIRE_READ_REQUEST_BIT 0x08
#define ONEWIRE_DELAY_REQUEST_BIT 0x10
#define ONEWIRE_WRITE_REQUEST_BIT 0x20

#define ONEWIRE_WITHDATA_REQUEST_BITS 0x3C

#define ONEWIRE_CRC 0 //for OneWire.h: crc-functions are not used by Firmata

//default value for power:
#define ONEWIRE_POWER 1

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
#define ONEWIRE                 0x07 // pin configured for 1-wire
#define STEPPER                 0x08 // pin configured for stepper motor
#define ENCODER                 0x09 // pin configured for rotary encoders
#define FIRMATA_INPUT_PULLUP	0x10 // enable pullup resistors
#define IGNORE                  0x7F // pin configured to be ignored by digitalWrite and capabilityResponse
#define TOTAL_PIN_MODES         12

extern "C" {
	// callback function types
	typedef void(*callbackFunction)(byte, int);
	typedef void(*systemResetCallbackFunction)(void);
	typedef void(*stringCallbackFunction)(char*);
	typedef void(*sysexCallbackFunction)(byte command, byte argc, byte*argv);
}


// TODO make it a subclass of a generic Serial/Stream base class
class FirmataClass
{
public:
	FirmataClass();
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
	void sendString(const char *string);
	void sendString(byte command, const char *string);
	void sendSysex(byte command, byte bytec, byte *bytev);
	void write(byte c);
	/* attach & detach callback functions to messages */
	void attach(byte command, callbackFunction newFunction);
	void attach(byte command, systemResetCallbackFunction newFunction);
	void attach(byte command, stringCallbackFunction newFunction);
	void attach(byte command, sysexCallbackFunction newFunction);
	void detach(byte command);

	/* utility methods */
	void sendValueAsTwo7bitBytes(int value);
	void startSysex(void);
	void endSysex(void);

private:
	Stream *FirmataStream;
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
	void strobeBlinkPin(int count, int onInterval, int offInterval);
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
  Encoder7Bit.h - Firmata library
  Copyright (C) 2012-2013 Norbert Truchsess. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef Encoder7Bit_h
#define Encoder7Bit_h
#include <Arduino.h>

#define num7BitOutbytes(a)(((a)*7)>>3)

class Encoder7BitClass
{
public:
  Encoder7BitClass();
  void startBinaryWrite();
  void endBinaryWrite();
  void writeBinary(byte data);
  void readBinary(int outBytes,byte *inData, byte *outData);

private:
  byte previous;
  int shift;
};

extern Encoder7BitClass Encoder7Bit;

#endif