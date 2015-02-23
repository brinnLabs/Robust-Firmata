/*
 * Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <list>
#include <vector>
#include <string>
#include <iostream>

#include "ofEvents.h"

#include "ofSerial.h"

/*
* Version numbers for the protocol. The protocol is still changing, so these
* version numbers are important. This number can be queried so that host
* software can test whether it will be compatible with the currently installed firmware.
*/

#define FIRMATA_MAJOR_VERSION                           2 // for non-compatible changes
#define FIRMATA_MINOR_VERSION                           4 // for backwards compatible changes
#define FIRMATA_MAX_DATA_BYTES                          32 // max number of data bytes in non-Sysex messages
// message command bytes (128-255/0x80-0xFF)
#define FIRMATA_DIGITAL_MESSAGE                         0x90 // send data for a digital pin
#define FIRMATA_ANALOG_MESSAGE                          0xE0 // send data for an analog pin (or PWM)
#define FIRMATA_REPORT_ANALOG                           0xC0 // enable analog input by pin #
#define FIRMATA_REPORT_DIGITAL                          0xD0 // enable digital input by port pair
//
#define FIRMATA_SET_PIN_MODE                            0xF4 // set a pin to INPUT/OUTPUT/PWM/etc
//
#define FIRMATA_REPORT_VERSION                          0xF9 // report protocol version
#define FIRMATA_SYSTEM_RESET                            0xFF // reset from MIDI
//
#define FIRMATA_START_SYSEX                             0xF0 // start a MIDI Sysex message
#define FIRMATA_END_SYSEX                               0xF7 // end a MIDI Sysex message
// pin modes
#define FIRMATA_INPUT                                   0x00
#define FIRMATA_OUTPUT                                  0x01
#define FIRMATA_ANALOG                                  0x02 // analog pin in analogInput mode
#define FIRMATA_PWM                                     0x03 // digital pin in PWM output mode
#define FIRMATA_SERVO                                   0x04 // digital pin in Servo output mode
#define SHIFT											0x05 // shiftIn/shiftOut mode
#define I2C												0x06 // pin included in I2C setup
#define ONEWIRE											0x07 // pin configured for 1-wire
#define STEPPER											0x08 // pin configured for stepper motor
#define ENCODER											0x09 // pin configured for rotary encoders
#define FIRMATA_INPUT_PULLUP							0x10 // enable pullup resistors
#define FIRMATA_IGNORE									0x7F
#define UNKOWN											0x1F

#define TOTAL_PIN_MODES 								13

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */
#define ENCODER_DATA									0x61 // reply with encoders current positions
#define FIRMATA_SYSEX_SERVO_CONFIG      				0x70 // set max angle, minPulse, maxPulse, freq
#define FIRMATA_SYSEX_FIRMATA_STRING					0x71 // a string message with 14-bits per char
#define STEPPER_DATA									0x72 // control a stepper motor
#define ONEWIRE_DATA									0x73 // send an OneWire read/write/reset/select/skip/search request
#define SHIFT_DATA										0x75 // a bitstream to/from a shift register
#define I2C_REQUEST										0x76 // send an I2C read/write request
#define I2C_REPLY										0x77 // a reply to an I2C read request
#define I2C_CONFIG										0x78 // config I2C settings such as delay times and power pins
#define EXTENDED_ANALOG									0x6F // analog write (PWM, Servo, etc) to any pin
#define PIN_STATE_QUERY									0x6D // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE								0x6E // reply with pin's current mode and value
#define CAPABILITY_QUERY								0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE								0x6C // reply with supported modes and resolution
#define ANALOG_MAPPING_QUERY							0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE							0x6A // reply with mapping info
#define FIRMATA_SYSEX_REPORT_FIRMWARE					0x79 // report name and version of the firmware
#define SAMPLING_INTERVAL								0x7A // set the poll rate of the main loop 
#define SCHEDULER_DATA									0x7B // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
#define FIRMATA_SYSEX_NON_REALTIME      				0x7E // MIDI Reserved for non-realtime messages
#define FIRMATA_SYSEX_REALTIME         					0x7F // MIDI Reserved for realtime messages
// these are DEPRECATED to make the naming more consistent
#define FIRMATA_STRING									0x71 // same as STRING_DATA
#define SYSEX_I2C_REQUEST								0x76 // same as I2C_REQUEST
#define SYSEX_I2C_REPLY									0x77 // same as I2C_REPLY
#define SYSEX_SAMPLING_INTERVAL							0x7A // same as SAMPLING_INTERVAL


#define ONEWIRE_CONFIG_REQUEST							0x41
#define ONEWIRE_DATA 									0x73
#define ONEWIRE_DELAY_REQUEST_BIT 						0x10
#define ONEWIRE_READ_REPLY								0x43
#define ONEWIRE_READ_REQUEST_BIT 						0x08
#define ONEWIRE_RESET_REQUEST_BIT 						0x01
#define ONEWIRE_SEARCH_ALARMS_REPLY 					0x45
#define ONEWIRE_SEARCH_ALARMS_REQUEST 					0x44
#define ONEWIRE_SEARCH_REPLY 							0x42
#define ONEWIRE_SEARCH_REQUEST							0x40
#define ONEWIRE_WITHDATA_REQUEST_BITS 					0x3C
#define ONEWIRE_WRITE_REQUEST_BIT						0x20

#define PULSE_IN										0x74
#define PULSE_OUT										0x73

// board settings
#define TOTAL_DIGITAL_PINS								120 // total number of pins currently supported
#define TOTAL_ANALOG_PINS								8
#define TOTAL_PORTS										15 // total number of ports for the board

// pin modes
#define ARD_INPUT                                       0x00
#define ARD_OUTPUT                                      0x01
#define ARD_ANALOG                                      0x02 // analog pin in analogInput mode
#define ARD_PWM                                         0x03 // digital pin in PWM output mode
#define ARD_SERVO                                       0x04 // digital pin in Servo output mode
#define ARD_SHIFT                   					0x05 // shiftIn/shiftOut mode
#define ARD_I2C                     					0x06 // pin included in I2C setup
#define ARD_ONEWIRE										0x07 // pin configured for 1-wire
#define ARD_STEPPER         							0x08  // pin configured for stepper motor
#define ARD_ENCODER										0x09 // pin configured for rotary encoders
#define ARD_INPUT_PULLUP								0x10 // enable pullup resistors
#define ARD_IGNORE										0x7F
#define UNKOWN											0x1F
#define ARD_HIGH                                        1
#define ARD_LOW                                         0
#define ARD_ON                                          1
#define ARD_OFF                                         0

#define MAX_STEPPERS    								6     // arbitrary value... may need to adjust
#define STEPPER_DATA    								0x72  // move this to Firmata.h
#define STEPPER_CONFIG  								0
#define STEPPER_STEP    								1
#define STEPPER_GET_POSITION							2
#define STEPPER_GET_DISTANCE_TO							3
#define STEPPER_SET_SPEED								4
#define STEPPER_SET_ACCEL								5
#define STEPPER_SET_DECEL								6
#define STEPPER_DONE									7
#define STEPPER_HOME									8
#define STEPPER_SET_HOME								9

#define SYSEX_SERVO_ATTACH								0x00
#define SYSEX_SERVO_DETACH								0x01
#define SYSEX_SERVO_WRITE								0x02

#define I2C_WRITE 										B00000000
#define I2C_READ 										B00001000
#define I2C_READ_CONTINUOUSLY 							B00010000
#define I2C_STOP_READING 								B00011000
#define I2C_READ_WRITE_MODE_MASK 						B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK						B00100000

#define MAX_ENCODERS									5 // arbitrary value, may need to adjust
#define ENCODER_ATTACH									0x00
#define ENCODER_REPORT_POSITION							0x01
#define ENCODER_REPORT_POSITIONS						0x02
#define ENCODER_RESET_POSITION							0x03
#define ENCODER_REPORT_AUTO								0x04
#define ENCODER_DETACH									0x05
#define ENCODER_DIRECTION_MASK							0x40 // B01000000
#define ENCODER_CHANNEL_MASK							0x3F // B00111111

#define MAX_QUERIES										8 // max number of i2c devices in read continuous mode
#define MINIMUM_SAMPLING_INTERVAL						10
#define REGISTER_NOT_SPECIFIED							-1

#define OF_ARDUINO_DELAY_LENGTH							10.0

enum Stepper_Interface {
	DRIVER =	1,
	TWO_WIRE =	2,
	FOUR_WIRE = 3
};

enum Direction {
	CCW =	0,
	CW =	1
};
enum i2c_modes {
	WRITE =				0x00,
	READ =				1,
	CONTINUOUS_READ =	2,
	STOP_READING =		3
};

struct I2C_Data{
	int		address;
	int		reg;
	string	data;
};

struct Encoder_Data{
	int		ID;
	bool	direction;
	int		position;
};

struct Stepper_Data{
	int		id;
	int		type;
	int		data;
};


class ofArduino{

public:
	ofArduino();

	virtual ~ofArduino();


	// --- setup functions
	bool connect(string device, int baud = 57600);
	// opens a serial port connection to the arduino

	void disconnect();
	// closes the serial port connection

	bool isArduinoReady();

	void reset();

	void  setUseDelay(bool bDelay);

	void update();
	// polls data from the serial port, this has to be called periodically

	void flushAll();

	bool isInitialized();
	// returns true if a succesfull connection has been established and the Wiring has reported a firmware

	void setDigitalHistoryLength(int length);
	void setAnalogHistoryLength(int length);
	void setStringHistoryLength(int length);
	void setSysExHistoryLength(int nSysEx);

	// --- senders

	void sendDigitalPinMode(int pin, int mode);

	void sendAnalogPinReporting(int pin, int mode);

	void sendDigital(int pin, int value, bool force = false);

	void sendPwm(int pin, int value, bool force = false);

	//void sendPwmServo(int pin, int value, bool force = false);

	void sendSysEx(int command, vector<unsigned char> data);

	void sendString(string str);
	// firmata can not handle strings longer than 12 characters.

	void sendProtocolVersionRequest();

	void sendFirmwareVersionRequest();

	void sendReset();

	// --- senders for SysEx communication

	void sendSysExBegin();
	// sends the FIRMATA_START_SYSEX command

	void sendSysExEnd();
	// sends the FIRMATA_END_SYSEX command

	void sendByte(unsigned char byte);
	// sends a byte without wrapping it in a firmata message, data has to be in the 0-127 range,
	// values > 127 will be interpreted as commands.

	void sendValueAsTwo7bitBytes(int value);
	// sends a value as two 7-bit bytes without wrapping it in a firmata message
	// values in the range 0 - 16384 will be sent as two bytes within the 0-127 data range.

	// --- getters

	int getPwm(int pin);

	int getDigital(int pin);

	int getAnalog(int pin);
	// returns the last received analog value (0-1023) for the given pin

	vector<unsigned char> getSysEx();
	// returns the last received SysEx message

	string getString();
	// returns the last received string

	int getMajorProtocolVersion();
	// returns the major firmware version

	int getMinorProtocolVersion();
	// returns the minor firmware version

	int getMajorFirmwareVersion();
	// returns the major firmware version

	int getMinorFirmwareVersion();
	// returns the minor firmware version

	string getFirmwareName();
	// returns the name of the firmware

	list<int>* getDigitalHistory(int pin);
	// returns a pointer to the digital data history list for the given pin

	list<int>* getAnalogHistory(int pin);
	// returns a pointer to the analog data history list for the given pin

	list<vector<unsigned char> >* getSysExHistory();
	// returns a pointer to the SysEx history

	list<string>* getStringHistory();
	// returns a pointer to the string history

	int getDigitalPinMode(int pin);
	// returns INPUT, OUTPUT, PWM, SERVO, ANALOG

	int getAnalogPinReporting(int pin);
	// returns ON, OFF

	int getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb);
	// useful for parsing SysEx messages

	// --- events

	ofEvent<const int> EDigitalPinChanged;
	// triggered when a digital pin changes value, the pin that changed is passed as an argument

	ofEvent<const int> EAnalogPinChanged;
	// triggered when an analog pin changes value, the pin that changed is passed as an argument

	ofEvent<const vector<unsigned char> > ESysExReceived;
	// triggered when a SysEx message that isn't in the extended command set is received, the SysEx message is passed as an argument

	ofEvent<const int> EProtocolVersionReceived;
	// triggered when a protocol version is received, the major version is passed as an argument

	ofEvent<const int> EFirmwareVersionReceived;
	// triggered when a firmware version is received, the major version is passed as an argument

	ofEvent<const int> EInitialized;
	// triggered when the firmware version is received upon connect, the major firmware version is passed as an argument
	// from this point it's safe to send to the Wiring.

	ofEvent<const string> EStringReceived;
	// triggered when a string is received, the string is passed as an argument

	ofEvent<const Stepper_Data> EStepperDataRecieved;
	// triggered when a stepper has finished rotating. Returns which stepper has complted its rotation

	ofEvent<const I2C_Data> EI2CDataRecieved;
	// triggered when the I2C bus returns data after a read request

 	ofEvent<const vector<Encoder_Data> > EEncoderDataRecieved;
	// triggered when the encoder returns data after a read request

	// -- stepper
	void sendStepper2Wire(int dirPin, int stepPin, int stepsPerRev = 200, int limitSwitch1 = 0, int limitSwitch2 = 0, bool switch1UsesPullup = true, bool switch2UsesPullup = true);
	// the pins has to have a stepper attached

	void sendStepper4Wire(int pin1, int pin2, int pin3, int pin4, int stepsPerRev = 200, int limitSwitch1 = 0, int limitSwitch2 = 0, bool switch1UsesPullup = true, bool switch2UsesPullup = true);
	// the pins has to have a stepper attached

	void sendStepperMove(int stepperID, int direction, int steps, int speed=0, float acceleration = 0, float deceleration = 0);
	// set the steps to move

	void getStepperPosition(int stepperID);
	// get the steppers current position

	void getStepperDistanceFrom(int stepperID);
	//get the steppers current distance from the target

	void setStepperSpeed(int stepperID, unsigned int speed);
	// set the steppers current speed in steps per second

	void setStepperAcceleration(int stepperID, unsigned int accel);
	// set the acceleration in steps per second per second

	void setStepperDeceleration(int stepperID, unsigned int decel);
	// set the deceleration in steps per second per second


	// -- servo
	void sendServo(int pin, int value, bool force = false);
	// the pin has to have a servo attached

	void sendServoAttach(int pin, int minPulse = 544, int maxPulse = 2400, int angle = 180);
	// attaches a servo to a pin

	void sendServoDetach(int pin);
	// detaches a servo from a pin, the pin mode remains as OUTPUT

	int getServo(int pin);
	// returns the last set servo value for a pin if the pin has a servo attached

	void sendI2CConfig(int delay);
	bool isI2CConfigured();
	void sendI2CWriteRequest(char slaveAddress, unsigned char * bytes, int numOfBytes);
	void sendI2CWriteRequest(char slaveAddress, vector<char> bytes);
	void i2cWrite(char address, unsigned char * bytes, int numOfBytes);
	void i2cWriteReg(char address, int reg, int byte);
	void sendI2CReadRequest(char address, unsigned char numBytes);
	void i2cRead(char address, unsigned char reg, int bytesToRead);
	void i2cReadOnce(char address, unsigned char reg, int bytesToRead);

	void sendOneWireConfig(int pin, bool enableParasiticPower);
	void sendOneWireSearch(int pin);
	void sendOneWireAlarmsSearch(int pin);
	void sendOneWireSearch(char type, int pin);
	void sendOneWireRead(int pin, unsigned char device, int numBytesToRead);
	void sendOneWireReset(int pin);
	void sendOneWireWrite(int pin, unsigned char device, unsigned char * data);
	void sendOneWireDelay(int pin, int delay);
	void sendOneWireWriteAndRead(int pin, unsigned char device, unsigned char * data, int numBytesToRead);
	void sendOneWireRequest(int pin, unsigned char subcommand, unsigned char device, int numBytesToRead, unsigned char correlationId, int delay, unsigned char * dataToWrite);

	void attachEncoder(int pinA, int pinB);
	void getEncoderPosition(int encoderNum);
	void getAllEncoderPositions();
	void resetEncoderPosition(int encoderNum);
	void enableEncoderReporting();
	void disableEncoderReporting();
	void detachEncoder(int encoderNum);

	bool isAttached();

protected:
	bool _initialized;

	void sendDigitalPinReporting(int pin, int mode);
	// sets pin reporting to ON or OFF
	// enables / disables reporting for the pins port

	void sendDigitalPortReporting(int port, int mode);
	// sets port reporting to ON or OFF
	// enables / disables reporting for ports

	void processData(unsigned char inputData);
	void processDigitalPort(int port, unsigned char value);
	virtual void processSysExData(vector<unsigned char> data);

	ofSerial _port;
	int _portStatus;

	// --- history variables
	int _analogHistoryLength;
	int _digitalHistoryLength;
	int _stringHistoryLength;
	int _sysExHistoryLength;

	// --- data processing variables
	int _waitForData;
	int _executeMultiByteCommand;
	int _multiByteChannel; // indicates which pin data came from

	// --- data holders
	unsigned char _storedInputData[FIRMATA_MAX_DATA_BYTES];
	vector<unsigned char> _sysExData;
	int _majorProtocolVersion;
	int _minorProtocolVersion;
	int _majorFirmwareVersion;
	int _minorFirmwareVersion;
	string _firmwareName;

	list<vector<unsigned char> > _sysExHistory;
	// maintains a history of received sysEx messages (excluding SysEx messages in the extended command set)

	list<string> _stringHistory;
	// maintains a history of received strings

	list<int> _analogHistory[TOTAL_ANALOG_PINS];
	// a history of received data for each analog pin

	list<int> _digitalHistory[TOTAL_DIGITAL_PINS];
	// a history of received data for each digital pin

	int _digitalPinMode[TOTAL_DIGITAL_PINS];
	// the modes for all digital pins

	int _digitalPinValue[TOTAL_DIGITAL_PINS];
	// the last set values (DIGITAL/PWM) on all digital pins

	int _digitalPortValue[TOTAL_PORTS];
	// the last set values on all ports

	int _digitalPortReporting[TOTAL_PORTS];
	// whether pin reporting is enabled / disabled

	int _digitalPinReporting[TOTAL_DIGITAL_PINS];
	// whether pin reporting is enabled / disabled

	int _analogPinReporting[TOTAL_ANALOG_PINS];
	// whether pin reporting is enabled / disabled

	bool bUseDelay;

	bool connected;

	float connectTime;

	bool _i2cConfigured;

	int _servoValue[TOTAL_DIGITAL_PINS];
	// the last set servo values

	int _numSteppers;

	int _numEncoders;

	int _stepperID;

	int _encoderID;

};

typedef ofArduino ofStandardFirmata;