/*
Firmata.cpp - Firmata library v2.4.0 - 2014-12-21
Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
See file LICENSE.txt for further informations on licensing terms.
*/

//******************************************************************************
//* Includes
//******************************************************************************

#include "Firmata.h"
#include "HardwareSerial.h"

extern "C" {
#include <string.h>
#include <stdlib.h>
}

//******************************************************************************
//* Support Functions
//******************************************************************************

void FirmataClass::sendValueAsTwo7bitBytes(int value)
{
	FirmataStream->write(value & B01111111); // LSB
	FirmataStream->write(value >> 7 & B01111111); // MSB
}

void FirmataClass::startSysex(void)
{
	FirmataStream->write(START_SYSEX);
}

void FirmataClass::endSysex(void)
{
	FirmataStream->write(END_SYSEX);
}

//******************************************************************************
//* Constructors
//******************************************************************************

FirmataClass::FirmataClass()
{
	firmwareVersionCount = 0;
	firmwareVersionVector = 0;
	systemReset();
}

//******************************************************************************
//* Public Methods
//******************************************************************************

/* begin method with default serial bitrate */
void FirmataClass::begin(void)
{
	begin(57600);
}

/* begin method for overriding default serial bitrate */
void FirmataClass::begin(long speed)
{
	Serial.begin(speed);
	FirmataStream = &Serial;
	blinkVersion();
	printVersion();
	printFirmwareVersion();
}

/* begin method for overriding default stream */
void FirmataClass::begin(Stream &s)
{
	FirmataStream = &s;
	// do not call blinkVersion() here because some hardware such as the
	// Ethernet shield use pin 13
	printVersion();
	printFirmwareVersion();
}

// output the protocol version message to the serial port
void FirmataClass::printVersion(void)
{
	FirmataStream->write(REPORT_VERSION);
	FirmataStream->write(FIRMATA_MAJOR_VERSION);
	FirmataStream->write(FIRMATA_MINOR_VERSION);
}

void FirmataClass::blinkVersion(void)
{
	// flash the pin with the protocol version
	pinMode(VERSION_BLINK_PIN, OUTPUT);
	strobeBlinkPin(FIRMATA_MAJOR_VERSION, 40, 210);
	delay(250);
	strobeBlinkPin(FIRMATA_MINOR_VERSION, 40, 210);
	delay(125);
}

void FirmataClass::printFirmwareVersion(void)
{
	byte i;

	if (firmwareVersionCount)  // make sure that the name has been set before reporting
	{
		startSysex();
		FirmataStream->write(REPORT_FIRMWARE);
		FirmataStream->write(firmwareVersionVector[0]); // major version number
		FirmataStream->write(firmwareVersionVector[1]); // minor version number
		for (i = 2; i < firmwareVersionCount; ++i)
		{
			sendValueAsTwo7bitBytes(firmwareVersionVector[i]);
		}
		endSysex();
	}
}

void FirmataClass::setFirmwareNameAndVersion(const char *name, byte major, byte minor)
{
	const char *firmwareName;
	const char *extension;

	// parse out ".cpp" and "applet/" that comes from using __FILE__
	extension = strstr(name, ".cpp");
	firmwareName = strrchr(name, '/');

	if (!firmwareName)
	{
		// windows
		firmwareName = strrchr(name, '\\');
	}
	if (!firmwareName)
	{
		// user passed firmware name
		firmwareName = name;
	}
	else
	{
		firmwareName++;
	}

	if (!extension)
	{
		firmwareVersionCount = strlen(firmwareName) + 2;
	}
	else
	{
		firmwareVersionCount = extension - firmwareName + 2;
	}

	// in case anyone calls setFirmwareNameAndVersion more than once
	free(firmwareVersionVector);

	firmwareVersionVector = (byte *)malloc(firmwareVersionCount);
	firmwareVersionVector[firmwareVersionCount] = 0;
	firmwareVersionVector[0] = major;
	firmwareVersionVector[1] = minor;
	strncpy((char *)firmwareVersionVector + 2, firmwareName, firmwareVersionCount - 2);
}

//------------------------------------------------------------------------------
// Serial Receive Handling

int FirmataClass::available(void)
{
	return FirmataStream->available();
}


void FirmataClass::processSysexMessage(void)
{
	switch (storedInputData[0])  //first byte in buffer is command
	{
	case REPORT_FIRMWARE:
		printFirmwareVersion();
		break;
	case STRING_DATA:
		if (currentStringCallback)
		{
			byte bufferLength = (sysexBytesRead - 1) / 2;
			byte i = 1;
			byte j = 0;
			while (j < bufferLength)
			{
				// The string length will only be at most half the size of the
				// stored input buffer so we can decode the string within the buffer.
				storedInputData[j] = storedInputData[i];
				i++;
				storedInputData[j] += (storedInputData[i] << 7);
				i++;
				j++;
			}
			// Make sure string is null terminated. This may be the case for data
			// coming from client libraries in languages that don't null terminate
			// strings.
			if (storedInputData[j - 1] != '\0')
			{
				storedInputData[j] = '\0';
			}
			(*currentStringCallback)((char *)&storedInputData[0]);
		}
		break;
	default:
		if (currentSysexCallback)
			(*currentSysexCallback)(storedInputData[0], sysexBytesRead - 1, storedInputData + 1);
	}
}

void FirmataClass::processInput(void)
{
	int inputData = FirmataStream->read(); // this is 'int' to handle -1 when no data
	int command;

	// TODO make sure it handles -1 properly

	if (parsingSysex)
	{
		if (inputData == END_SYSEX)
		{
			//stop sysex byte
			parsingSysex = false;
			//fire off handler function
			processSysexMessage();
		}
		else
		{
			//normal data byte - add to buffer
			storedInputData[sysexBytesRead] = inputData;
			sysexBytesRead++;
		}
	}
	else if ((waitForData > 0) && (inputData < 128))
	{
		waitForData--;
		storedInputData[waitForData] = inputData;
		if ((waitForData == 0) && executeMultiByteCommand) // got the whole message
		{
			switch (executeMultiByteCommand)
			{
			case ANALOG_MESSAGE:
				if (currentAnalogCallback)
				{
					(*currentAnalogCallback)(multiByteChannel,
						(storedInputData[0] << 7)
						+ storedInputData[1]);
				}
				break;
			case DIGITAL_MESSAGE:
				if (currentDigitalCallback)
				{
					(*currentDigitalCallback)(multiByteChannel,
						(storedInputData[0] << 7)
						+ storedInputData[1]);
				}
				break;
			case SET_PIN_MODE:
				if (currentPinModeCallback)
					(*currentPinModeCallback)(storedInputData[1], storedInputData[0]);
				break;
			case REPORT_ANALOG:
				if (currentReportAnalogCallback)
					(*currentReportAnalogCallback)(multiByteChannel, storedInputData[0]);
				break;
			case REPORT_DIGITAL:
				if (currentReportDigitalCallback)
					(*currentReportDigitalCallback)(multiByteChannel, storedInputData[0]);
				break;
			}
			executeMultiByteCommand = 0;
		}
	}
	else
	{
		// remove channel info from command byte if less than 0xF0
		if (inputData < 0xF0)
		{
			command = inputData & 0xF0;
			multiByteChannel = inputData & 0x0F;
		}
		else
		{
			command = inputData;
			// commands in the 0xF* range don't use channel data
		}
		switch (command)
		{
		case ANALOG_MESSAGE:
		case DIGITAL_MESSAGE:
		case SET_PIN_MODE:
			waitForData = 2; // two data bytes needed
			executeMultiByteCommand = command;
			break;
		case REPORT_ANALOG:
		case REPORT_DIGITAL:
			waitForData = 1; // one data byte needed
			executeMultiByteCommand = command;
			break;
		case START_SYSEX:
			parsingSysex = true;
			sysexBytesRead = 0;
			break;
		case SYSTEM_RESET:
			systemReset();
			break;
		case REPORT_VERSION:
			Firmata.printVersion();
			break;
		}
	}
}

//------------------------------------------------------------------------------
// Serial Send Handling

// send an analog message
void FirmataClass::sendAnalog(byte pin, int value)
{
	// pin can only be 0-15, so chop higher bits
	FirmataStream->write(ANALOG_MESSAGE | (pin & 0xF));
	sendValueAsTwo7bitBytes(value);
}

// send a single digital pin in a digital message
void FirmataClass::sendDigital(byte pin, int value)
{
	/* TODO add single pin digital messages to the protocol, this needs to
	* track the last digital data sent so that it can be sure to change just
	* one bit in the packet.  This is complicated by the fact that the
	* numbering of the pins will probably differ on Arduino, Wiring, and
	* other boards.  The DIGITAL_MESSAGE sends 14 bits at a time, but it is
	* probably easier to send 8 bit ports for any board with more than 14
	* digital pins.
	*/

	// TODO: the digital message should not be sent on the serial port every
	// time sendDigital() is called.  Instead, it should add it to an int
	// which will be sent on a schedule.  If a pin changes more than once
	// before the digital message is sent on the serial port, it should send a
	// digital message for each change.

	//    if(value == 0)
	//        sendDigitalPortPair();
}


// send 14-bits in a single digital message (protocol v1)
// send an 8-bit port in a single digital message (protocol v2)
void FirmataClass::sendDigitalPort(byte portNumber, int portData)
{
	FirmataStream->write(DIGITAL_MESSAGE | (portNumber & 0xF));
	FirmataStream->write((byte)portData % 128); // Tx bits 0-6
	FirmataStream->write(portData >> 7);  // Tx bits 7-13
}


void FirmataClass::sendSysex(byte command, byte bytec, byte *bytev)
{
	byte i;
	startSysex();
	FirmataStream->write(command);
	for (i = 0; i < bytec; i++)
	{
		sendValueAsTwo7bitBytes(bytev[i]);
	}
	endSysex();
}

void FirmataClass::sendString(byte command, const char *string)
{
	sendSysex(command, strlen(string), (byte *)string);
}


// send a string as the protocol string type
void FirmataClass::sendString(const char *string)
{
	sendString(STRING_DATA, string);
}

// expose the write method
void FirmataClass::write(byte c)
{
	FirmataStream->write(c);
}


// Internal Actions/////////////////////////////////////////////////////////////

// generic callbacks
void FirmataClass::attach(byte command, callbackFunction newFunction)
{
	switch (command)
	{
	case ANALOG_MESSAGE: currentAnalogCallback = newFunction; break;
	case DIGITAL_MESSAGE: currentDigitalCallback = newFunction; break;
	case REPORT_ANALOG: currentReportAnalogCallback = newFunction; break;
	case REPORT_DIGITAL: currentReportDigitalCallback = newFunction; break;
	case SET_PIN_MODE: currentPinModeCallback = newFunction; break;
	}
}

void FirmataClass::attach(byte command, systemResetCallbackFunction newFunction)
{
	switch (command)
	{
	case SYSTEM_RESET: currentSystemResetCallback = newFunction; break;
	}
}

void FirmataClass::attach(byte command, stringCallbackFunction newFunction)
{
	switch (command)
	{
	case STRING_DATA: currentStringCallback = newFunction; break;
	}
}

void FirmataClass::attach(byte command, sysexCallbackFunction newFunction)
{
	currentSysexCallback = newFunction;
}

void FirmataClass::detach(byte command)
{
	switch (command)
	{
	case SYSTEM_RESET: currentSystemResetCallback = NULL; break;
	case STRING_DATA: currentStringCallback = NULL; break;
	case START_SYSEX: currentSysexCallback = NULL; break;
	default:
		attach(command, (callbackFunction)NULL);
	}
}

// sysex callbacks
/*
* this is too complicated for analogReceive, but maybe for Sysex?
void FirmataClass::attachSysex(sysexFunction newFunction)
{
byte i;
byte tmpCount = analogReceiveFunctionCount;
analogReceiveFunction* tmpArray = analogReceiveFunctionArray;
analogReceiveFunctionCount++;
analogReceiveFunctionArray = (analogReceiveFunction*) calloc(analogReceiveFunctionCount, sizeof(analogReceiveFunction));
for(i = 0; i < tmpCount; i++) {
analogReceiveFunctionArray[i] = tmpArray[i];
}
analogReceiveFunctionArray[tmpCount] = newFunction;
free(tmpArray);
}
*/

//******************************************************************************
//* Private Methods
//******************************************************************************



// resets the system state upon a SYSTEM_RESET message from the host software
void FirmataClass::systemReset(void)
{
	byte i;

	waitForData = 0; // this flag says the next serial input will be data
	executeMultiByteCommand = 0; // execute this after getting multi-byte data
	multiByteChannel = 0; // channel data for multiByteCommands

	for (i = 0; i < MAX_DATA_BYTES; i++)
	{
		storedInputData[i] = 0;
	}

	parsingSysex = false;
	sysexBytesRead = 0;

	if (currentSystemResetCallback)
		(*currentSystemResetCallback)();
}



// =============================================================================
// used for flashing the pin for the version number
void FirmataClass::strobeBlinkPin(int count, int onInterval, int offInterval)
{
	byte i;
	pinMode(VERSION_BLINK_PIN, OUTPUT);
	for (i = 0; i < count; i++)
	{
		delay(offInterval);
		digitalWrite(VERSION_BLINK_PIN, HIGH);
		delay(onInterval);
		digitalWrite(VERSION_BLINK_PIN, LOW);
	}
}


// make one instance for the user to use
FirmataClass Firmata;

/**
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

/**
 * Constructor.
 *
 * Configure a stepper for an EasyDriver or other step + direction interface or
 * configure a bipolar or unipolar stepper motor for 2 wire drive mode.
 * Configure a bipolar or unipolar stepper for 4 wire drive mode.
 * @param interface The interface type: FirmataStepper::DRIVER or
 * FirmataStepper::TWO_WIRE
 * @param steps_per_rev The number of steps to make 1 revolution.
 * @param first_pin The direction pin (EasyDriver) or the pin attached to the
 * 1st motor coil (2 wire drive mode)
 * @param second_pin The step pin (EasyDriver) or the pin attached to the 2nd
 * motor coil (2 wire drive mode)
 * @param motor_pin_3 The pin attached to the 3rd motor coil
 * @param motor_pin_4 The pin attached to the 4th motor coil
 */
FirmataStepper::FirmataStepper(byte interface,
	int step_per_rev,
	byte pin1,
	byte pin2,
	byte pin3,
	byte pin4,
	byte lSwitchA,
	byte lSwitchB,
	bool lSwitchAType,
	bool lSwitchBType) {

	_step_number = 0;      // which step the motor is on
	_direction = 0;      // motor direction
	_last_step_time = 0;    // time stamp in ms of the last step taken
	_steps_per_rev = step_per_rev;    // total number of steps for this motor
	_running = false;
	_interface = interface; // default to Easy Stepper (or other step + direction driver)

	_motor_pin_1 = pin1;
	_motor_pin_2 = pin2;
	_dir_pin = pin1;
	_step_pin = pin2;

	_accel = 0;
	_decel = 0;
	_speed = 100;

	_areLimitSwitches = false;
	_limit_switch_a = -1;
	_limit_switch_b = -1;
	if (lSwitchA > 0 || lSwitchB > 0) {
		_areLimitSwitches = true;
		if (lSwitchA > 0){
			if (lSwitchAType){
				pinMode(lSwitchA, INPUT_PULLUP);
				_switch_a_type = false; //set to opposite state so we know when the switch is tripped LOW == 0/false
			}
			else {
				pinMode(lSwitchA, INPUT);
				_switch_a_type = true; //set to opposite state so we know when the switch is tripped HIGH == a/true
			}
			_limit_switch_a = lSwitchA;
		}
		if (lSwitchB > 0){
			if (lSwitchBType){
				pinMode(lSwitchB, INPUT_PULLUP);
				_switch_b_type = false; //set to opposite state so we know when the switch is tripped LOW == 0/false
			}
			else {
				pinMode(lSwitchB, INPUT);
				_switch_b_type = true; //set to opposite state so we know when the switch is tripped HIGH == a/true
			}
			_areLimitSwitches = true;
			_limit_switch_b = lSwitchB;
		}

	}
	// setup the pins on the microcontroller:
	pinMode(_motor_pin_1, OUTPUT);
	pinMode(_motor_pin_2, OUTPUT);

	if (_interface == FirmataStepper::FOUR_WIRE) {
		_motor_pin_3 = pin3;
		_motor_pin_4 = pin4;
		pinMode(_motor_pin_3, OUTPUT);
		pinMode(_motor_pin_4, OUTPUT);
	}


	_alpha = TWO_PI / _steps_per_rev;
	_at_x100 = (long)(_alpha * T1_FREQ * 100);
	_ax20000 = (long)(_alpha * 20000);
	_alpha_x2 = _alpha * 2;

}

long FirmataStepper::getPosition(){
	return _direction ? _stepCount : -_stepCount;
}
long FirmataStepper::getDistanceTo(){
	return (_steps_to_move - _stepCount) * (_direction ? 1 : -1);
}
void FirmataStepper::setSpeed(int speed){
	_speed = speed;
}
void FirmataStepper::setAcceleration(int accel){
	_accel = accel;
}
void FirmataStepper::setDeceleration(int decel){
	_decel = decel;
}

/**
 * Move the stepper a given number of steps at the specified
 * speed (rad/sec), acceleration (rad/sec^2) and deceleration (rad/sec^2).
 *
 * @param steps_to_move The number ofsteps to move the motor
 * @param speed [optional] Max speed in 0.01*rad/sec
 * @param accel [optional] Acceleration in 0.01*rad/sec^2
 * @param decel [optional] Deceleration in 0.01*rad/sec^2
 * for the optional parameters, we will never send a signed value so using -1
 * as a flag is safe
 */
void FirmataStepper::setStepsToMove(long steps_to_move, int speed, int accel, int decel) {
	unsigned long maxStepLimit;
	unsigned long accelerationLimit;

	_step_number = 0;
	_lastAccelDelay = 0;
	_stepCount = 0;
	_rest = 0;

	if (speed != -1)
		_speed = speed;
	if (accel != -1)
		_accel = accel;
	if (decel != -1)
		_decel = decel;

	if (steps_to_move < 0) {
		_direction = FirmataStepper::CCW;
		steps_to_move = -steps_to_move;
	}
	else {
		_direction = FirmataStepper::CW;
	}

	_steps_to_move = steps_to_move;

	// set max speed limit, by calc min_delay
	// min_delay = (alpha / tt)/w
	_min_delay = _at_x100 / _speed;

	// if acceleration or deceleration are not defined
	// start in RUN state and do no decelerate
	if (_accel == 0 || _decel == 0) {
		_step_delay = _min_delay;

		_decel_start = _steps_to_move;
		_run_state = FirmataStepper::RUN;
		_accel_count = 0;
		_running = true;

		return;
	}

	// if only moving 1 step
	if (_steps_to_move == 1) {

		// move one step
		_accel_count = -1;
		_run_state = FirmataStepper::DECEL;

		_step_delay = _min_delay;
		_running = true;
	}
	else if (_steps_to_move != 0) {
		// set initial step delay
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
		_step_delay = (long)((T1_FREQ_148 * sqrt(_alpha_x2 / _accel)) * 1000);

		// find out after how many steps does the speed hit the max speed limit.
		// maxSpeedLimit = speed^2 / (2*alpha*accel)
		maxStepLimit = (long)_speed*_speed / (long)(((long)_ax20000*_accel) / 100);

		// if we hit max spped limit before 0.5 step it will round to 0.
		// but in practice we need to move at least 1 step to get any speed at all.
		if (maxStepLimit == 0) {
			maxStepLimit = 1;
		}

		// find out after how many steps we must start deceleration.
		// n1 = (n1+n2)decel / (accel + decel)
		accelerationLimit = (long)((_steps_to_move*_decel) / (_accel + _decel));

		// we must accelerate at least 1 step before we can start deceleration
		if (accelerationLimit == 0) {
			accelerationLimit = 1;
		}

		// use the limit we hit first to calc decel
		if (accelerationLimit <= maxStepLimit) {
			_decel_val = accelerationLimit - _steps_to_move;
		}
		else {
			_decel_val = -(long)(maxStepLimit*_accel) / _decel;
		}

		// we must decelerate at least 1 step to stop
		if (_decel_val == 0) {
			_decel_val = -1;
		}

		// find step to start deceleration
		_decel_start = _steps_to_move + _decel_val;

		// if the max spped is so low that we don't need to go via acceleration state.
		if (_step_delay <= _min_delay) {
			_step_delay = _min_delay;
			_run_state = FirmataStepper::RUN;
		}
		else {
			_run_state = FirmataStepper::ACCEL;
		}

		// reset counter
		_accel_count = 0;
		_running = true;
	}
}


bool FirmataStepper::update() {
	bool done = false;
	long newStepDelay;

	if (_limit_switch_a > 0) {
		if (digitalRead(_limit_switch_a) == _switch_a_type){
			_a_tripped = true;
		}
		else {
			_a_tripped = false;
		}
	}
	if (_limit_switch_b > 0) {
		if (digitalRead(_limit_switch_b) == _switch_b_type){
			_b_tripped = true;
		}
		else {
			_b_tripped = false;
		}
	}


	unsigned long curTimeVal = micros();
	long timeDiff = curTimeVal - _last_step_time;

	if (_running == true && timeDiff >= _step_delay) {

		_last_step_time = curTimeVal;

		switch (_run_state) {
		case FirmataStepper::STOP:
			_stepCount = 0;
			_rest = 0;
			if (_running) {
				done = true;
			}
			_running = false;
			break;

		case FirmataStepper::ACCEL:
			updateStepPosition();
			_stepCount++;
			_accel_count++;
			newStepDelay = _step_delay - (((2 * (long)_step_delay) + _rest) / (4 * _accel_count + 1));
			_rest = ((2 * (long)_step_delay) + _rest) % (4 * _accel_count + 1);

			// check if we should start deceleration
			if (_stepCount >= _decel_start) {
				_accel_count = _decel_val;
				_run_state = FirmataStepper::DECEL;
				_rest = 0;
			}
			// check if we hit max speed
			else if (newStepDelay <= _min_delay) {
				_lastAccelDelay = newStepDelay;
				newStepDelay = _min_delay;
				_rest = 0;
				_run_state = FirmataStepper::RUN;
			}
			break;

		case FirmataStepper::RUN:
			updateStepPosition();
			_stepCount++;
			newStepDelay = _min_delay;

			// if no accel or decel was specified, go directly to STOP state
			if (_stepCount >= _steps_to_move) {
				_run_state = FirmataStepper::STOP;
			}
			// check if we should start deceleration
			else if (_stepCount >= _decel_start) {
				_accel_count = _decel_val;
				// start deceleration with same delay that accel ended with
				newStepDelay = _lastAccelDelay;
				_run_state = FirmataStepper::DECEL;
			}
			break;

		case FirmataStepper::DECEL:
			updateStepPosition();
			_stepCount++;
			_accel_count++;

			newStepDelay = _step_delay - (((2 * (long)_step_delay) + _rest) / (4 * _accel_count + 1));
			_rest = ((2 * (long)_step_delay) + _rest) % (4 * _accel_count + 1);

			if (newStepDelay < 0) newStepDelay = -newStepDelay;
			// check if we are at the last step
			if (_accel_count >= 0) {
				_run_state = FirmataStepper::STOP;
			}

			break;
		}

		_step_delay = newStepDelay;

	}

	return done;

}

/**
 * Update the step position.
 * @private
 */
void FirmataStepper::updateStepPosition() {
	// increment or decrement the step number,
	// depending on direction:
	if (_direction == FirmataStepper::CW) {
		_step_number++;
		if (_step_number >= _steps_per_rev) {
			_step_number = 0;
		}
	}
	else {
		if (_step_number <= 0) {
			_step_number = _steps_per_rev;
		}
		_step_number--;
	}

	// step the motor to step number 0, 1, 2, or 3:
	stepMotor(_step_number % 4, _direction);
}

/**
 * Moves the motor forward or backwards.
 * @param step_num For 2 or 4 wire configurations, this is the current step in
 * the 2 or 4 step sequence.
 * @param direction The direction of rotation
 */
void FirmataStepper::stepMotor(byte step_num, byte direction) {
	//since this will run regardless asyncronously we need to have it check for limit switches
	//switch a is assumed to be in the LOW direction, b in the high
	//are there limit switches? are they tripped? is the direction of movement towards them
	if (!_areLimitSwitches || (!_a_tripped && !_b_tripped) || ((_a_tripped && direction == 1) || (_b_tripped && direction == 0))){
		if (_interface == FirmataStepper::DRIVER) {
			digitalWrite(_dir_pin, direction);
			delayMicroseconds(1);
			digitalWrite(_step_pin, LOW);
			delayMicroseconds(1);
			digitalWrite(_step_pin, HIGH);
		}
		else if (_interface == FirmataStepper::TWO_WIRE) {
			switch (step_num) {
			case 0: /* 01 */
				digitalWrite(_motor_pin_1, LOW);
				digitalWrite(_motor_pin_2, HIGH);
				break;
			case 1: /* 11 */
				digitalWrite(_motor_pin_1, HIGH);
				digitalWrite(_motor_pin_2, HIGH);
				break;
			case 2: /* 10 */
				digitalWrite(_motor_pin_1, HIGH);
				digitalWrite(_motor_pin_2, LOW);
				break;
			case 3: /* 00 */
				digitalWrite(_motor_pin_1, LOW);
				digitalWrite(_motor_pin_2, LOW);
				break;
			}
		}
		else if (_interface == FirmataStepper::FOUR_WIRE) {
			switch (step_num) {
			case 0:    // 1010
				digitalWrite(_motor_pin_1, HIGH);
				digitalWrite(_motor_pin_2, LOW);
				digitalWrite(_motor_pin_3, HIGH);
				digitalWrite(_motor_pin_4, LOW);
				break;
			case 1:    // 0110
				digitalWrite(_motor_pin_1, LOW);
				digitalWrite(_motor_pin_2, HIGH);
				digitalWrite(_motor_pin_3, HIGH);
				digitalWrite(_motor_pin_4, LOW);
				break;
			case 2:    //0101
				digitalWrite(_motor_pin_1, LOW);
				digitalWrite(_motor_pin_2, HIGH);
				digitalWrite(_motor_pin_3, LOW);
				digitalWrite(_motor_pin_4, HIGH);
				break;
			case 3:    //1001
				digitalWrite(_motor_pin_1, HIGH);
				digitalWrite(_motor_pin_2, LOW);
				digitalWrite(_motor_pin_3, LOW);
				digitalWrite(_motor_pin_4, HIGH);
				break;
			}
		}
	}
}

/**
 * @return The version number of this library.
 */
byte FirmataStepper::version(void) {
	return 1;
}

/*
  Encoder7Bit.cpp - Firmata library
  Copyright (C) 2012-2013 Norbert Truchsess. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
  */

Encoder7BitClass::Encoder7BitClass()
{
	previous = 0;
	shift = 0;
}

void Encoder7BitClass::startBinaryWrite()
{
	shift = 0;
}

void Encoder7BitClass::endBinaryWrite()
{
	if (shift > 0) {
		Firmata.write(previous);
	}
}

void Encoder7BitClass::writeBinary(byte data)
{
	if (shift == 0) {
		Firmata.write(data & 0x7f);
		shift++;
		previous = data >> 7;
	}
	else {
		Firmata.write(((data << shift) & 0x7f) | previous);
		if (shift == 6) {
			Firmata.write(data >> 1);
			shift = 0;
		}
		else {
			shift++;
			previous = data >> (8 - shift);
		}
	}
}

void Encoder7BitClass::readBinary(int outBytes, byte *inData, byte *outData)
{
	for (int i = 0; i < outBytes; i++) {
		int j = i << 3;
		int pos = j / 7;
		byte shift = j % 7;
		outData[i] = (inData[pos] >> shift) | ((inData[pos + 1] << (7 - shift)) & 0xFF);
	}
}

Encoder7BitClass Encoder7Bit;