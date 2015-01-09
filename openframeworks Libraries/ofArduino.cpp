/*
* Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
* Wiring version 2011 (c) Carlos Mario Rodriguez and Hernando Barragan
*
* Devloped at: The Interactive Institutet / Art and Technology,
* OF Lab / Ars Electronica
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
* included in all copies or substantial _portions of the Software.
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

#include "ofArduino.h"
#include "ofUtils.h"

// TODO thread it?
// TODO throw event or exception if the serial port goes down...
//---------------------------------------------------------------------------
ofArduino::ofArduino(){
	_i2cConfigured = false;
	_portStatus = -1;
	_waitForData = 0;
	_analogHistoryLength = 2;
	_digitalHistoryLength = 2;
	_stringHistoryLength = 1;
	_sysExHistoryLength = 1;
	_numSteppers = 0;
	_numEncoders = 0;
	_stepperID = 0;
	_encoderID = 0;

	_majorProtocolVersion = 0;
	_minorProtocolVersion = 0;
	_majorFirmwareVersion = 0;
	_minorFirmwareVersion = 0;
	_firmwareName = "Unknown";

	// ports
	for (int i = 0; i < TOTAL_PORTS; ++i) {
		_digitalPortValue[i] = 0;
		_digitalPortReporting[i] = ARD_OFF;
	}

	// digital pins
	for (int i = 0; i < TOTAL_DIGITAL_PINS; ++i) {
		_digitalPinValue[i] = -1;
		_digitalPinMode[i] = ARD_OUTPUT;
		_digitalPinReporting[i] = ARD_OFF;
	}

	// analog in pins
	for (int i = 0; i < TOTAL_ANALOG_PINS; ++i) {
		_analogPinReporting[i] = ARD_OFF;
		// analog pins used as digital
		_digitalPinMode[i] = ARD_ANALOG;
		_digitalPinValue[i] = -1;
	}
	for (int i = 0; i < TOTAL_DIGITAL_PINS; ++i) {
		_servoValue[i] = -1;
	}
	bUseDelay = true;
	//sendFirmwareVersionRequest();
}

ofArduino::~ofArduino() {
	_port.close();
}

bool ofArduino::connect(string device, int baud){
	connectTime = ofGetElapsedTimef();
	_initialized = false;
	_port.enumerateDevices();
	connected = _port.setup(device.c_str(), baud);
	sendFirmwareVersionRequest();
	return connected;
}

bool ofArduino::isArduinoReady(){
	if (bUseDelay)
		return (ofGetElapsedTimef() - connectTime) > OF_ARDUINO_DELAY_LENGTH ? connected : false;
	else
		return connected;
}

void  ofArduino::setUseDelay(bool bDelay){
	bUseDelay = bDelay;
}

void ofArduino::setDigitalHistoryLength(int length){
	if (length >= 2)
		_digitalHistoryLength = length;
}

void ofArduino::setAnalogHistoryLength(int length){
	if (length >= 2)
		_analogHistoryLength = length;
}

void ofArduino::setSysExHistoryLength(int length){
	if (length >= 1)
		_sysExHistoryLength = length;
}

void ofArduino::setStringHistoryLength(int length){
	if (length >= 1)
		_stringHistoryLength = length;
}

void ofArduino::disconnect(){
	_port.close();
}

void ofArduino::update(){
<<<<<<< HEAD
	
=======

>>>>>>> origin/experimental
	//the computer should be able to read the entire buffer faster than it can be filled
	while (_port.available()) {

		int byte = _port.readByte();

		// process data....
		if (byte != OF_SERIAL_ERROR || byte != OF_SERIAL_NO_DATA) {
			processData((char)(byte));
		}
		// _port buffer is empty
		else{
			break;
		}
	}
<<<<<<< HEAD
	
=======

>>>>>>> origin/experimental
}

int ofArduino::getAnalog(int pin){
	if (_analogHistory[pin].size() > 0)
		return _analogHistory[pin].front();
	else
		return -1;
}

int ofArduino::getDigital(int pin){
	if ((_digitalPinMode[pin] == ARD_INPUT || _digitalPinMode[pin] == ARD_INPUT_PULLUP) && _digitalHistory[pin].size() > 0)
		return _digitalHistory[pin].front();
	else if (_digitalPinMode[pin] == ARD_OUTPUT)
		return _digitalPinValue[pin];
	else
		return -1;
}

int ofArduino::getPwm(int pin){
	if (_digitalPinMode[pin] == ARD_PWM)
		return _digitalPinValue[pin];
	else
		return -1;
}

vector<unsigned char> ofArduino::getSysEx(){
	return _sysExHistory.front();
}

string ofArduino::getString(){
	return _stringHistory.front();
}

int ofArduino::getDigitalPinMode(int pin){
	return _digitalPinMode[pin];
}

void ofArduino::sendDigital(int pin, int value, bool force){
	if ((_digitalPinMode[pin] == ARD_INPUT || _digitalPinMode[pin] == ARD_INPUT_PULLUP || _digitalPinMode[pin] == ARD_OUTPUT) && (_digitalPinValue[pin] != value || force)){

		_digitalPinValue[pin] = value;

		// set the bit
		int port = (pin >> 3) & 0x0F;

		if (value == 1)
			_digitalPortValue[port] |= (1 << (pin & 0x07));

		// clear the bit
		if (value == 0)
			_digitalPortValue[port] &= ~(1 << (pin & 0x07));

		sendByte(FIRMATA_DIGITAL_MESSAGE | port);
		sendValueAsTwo7bitBytes(_digitalPortValue[port]);

	}
}

void ofArduino::sendPwm(int pin, int value, bool force){
	if (_digitalPinMode[pin] == ARD_PWM && (_digitalPinValue[pin] != value || force)){
		sendByte(FIRMATA_ANALOG_MESSAGE | (pin & 0x0F));
		sendValueAsTwo7bitBytes(value);
		_digitalPinValue[pin] = value;
	}
	else if (_digitalPinMode[pin] == ARD_SERVO && (_servoValue[pin] != value || force)){
		sendByte(FIRMATA_ANALOG_MESSAGE | (pin & 0x0F));
		sendValueAsTwo7bitBytes(value);
		_servoValue[pin] = value;
	}
}

/*void ofArduino::sendPwmServo(int pin, int value, bool force){
if(_digitalPinMode[pin]==ARD_SERVO && (_servoValue[pin]!=value || force)){
sendByte(FIRMATA_ANALOG_MESSAGE | (pin & 0x0F));
sendValueAsTwo7bitBytes(value);
_servoValue[pin] = value;
}
}*/


void ofArduino::sendSysEx(int command, vector<unsigned char> data){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(command);
	vector<unsigned char>::iterator it = data.begin();
	while (it != data.end()) {
		sendByte(*it);
		it++;
	}
	sendByte(FIRMATA_END_SYSEX);
}

void ofArduino::sendSysExBegin(){
	sendByte(FIRMATA_START_SYSEX);
}

void ofArduino::sendSysExEnd(){
	sendByte(FIRMATA_END_SYSEX);
}

void ofArduino::sendString(string str){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(FIRMATA_SYSEX_FIRMATA_STRING);
	string::iterator it = str.begin();
	while (it != str.end()) {
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendByte(FIRMATA_END_SYSEX);
}

void ofArduino::sendProtocolVersionRequest(){
	sendByte(FIRMATA_REPORT_VERSION);
}

void ofArduino::sendFirmwareVersionRequest(){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(FIRMATA_SYSEX_REPORT_FIRMWARE);
	sendByte(FIRMATA_END_SYSEX);
}

void ofArduino::sendReset(){
	sendByte(FIRMATA_SYSTEM_RESET);
}

void ofArduino::sendAnalogPinReporting(int pin, int mode){
	sendByte(FIRMATA_REPORT_ANALOG | pin);
	sendByte(mode);
	_analogPinReporting[pin] = mode;
}

void ofArduino::sendDigitalPinMode(int pin, int mode){
	sendByte(FIRMATA_SET_PIN_MODE);
	sendByte(pin); // Tx pins 0-6
	sendByte(mode);
	_digitalPinMode[pin] = mode;

	// turn on or off reporting on the port

	if (mode == ARD_INPUT || mode == ARD_INPUT_PULLUP){
		sendDigitalPinReporting(pin, ARD_ON);
	}
	else {
		sendDigitalPinReporting(pin, ARD_OFF);
	}
}

int ofArduino::getAnalogPinReporting(int pin){
	return _analogPinReporting[pin];
}

list<int>* ofArduino::getAnalogHistory(int pin){
	return &_analogHistory[pin];
}

list<int>* ofArduino::getDigitalHistory(int pin){
	return &_digitalHistory[pin];
}

list<vector<unsigned char> >* ofArduino::getSysExHistory(){
	return &_sysExHistory;
}

list<string>* ofArduino::getStringHistory(){
	return &_stringHistory;
}

int ofArduino::getMajorProtocolVersion(){
	return _majorFirmwareVersion;
}

int ofArduino::getMinorProtocolVersion(){
	return _minorFirmwareVersion;
}

int ofArduino::getMajorFirmwareVersion(){
	return _majorFirmwareVersion;
}

int ofArduino::getMinorFirmwareVersion(){
	return _minorFirmwareVersion;
}

string ofArduino::getFirmwareName(){
	return _firmwareName;
}

bool ofArduino::isInitialized(){
	return _initialized;
}

// ------------------------------ private functions

void ofArduino::processData(unsigned char inputData){


	char msg[100];
	sprintf(msg, "Received Byte: %i", inputData);
	//Logger::get("Application").information(msg);

	// we have command data
	if (_waitForData > 0 && inputData < 128) {
		_waitForData--;

		// collect the data
		_storedInputData[_waitForData] = inputData;

		// we have all data executeMultiByteCommand
		if (_waitForData == 0) {
			switch (_executeMultiByteCommand) {
			case FIRMATA_DIGITAL_MESSAGE:
				processDigitalPort(_multiByteChannel, (_storedInputData[0] << 7) | _storedInputData[1]);
				break;
			case FIRMATA_REPORT_VERSION: // report version
				_majorProtocolVersion = _storedInputData[1];
				_minorProtocolVersion = _storedInputData[0];
				ofNotifyEvent(EProtocolVersionReceived, _majorProtocolVersion, this);
				break;
			case FIRMATA_ANALOG_MESSAGE:
				if (_analogHistory[_multiByteChannel].size() > 0){
					int previous = _analogHistory[_multiByteChannel].front();

					_analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					if ((int)_analogHistory[_multiByteChannel].size() > _analogHistoryLength)
						_analogHistory[_multiByteChannel].pop_back();

					// trigger an event if the pin has changed value
					if (_analogHistory[_multiByteChannel].front() != previous)
						ofNotifyEvent(EAnalogPinChanged, _multiByteChannel, this);
				}
				else{
					_analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					if ((int)_analogHistory[_multiByteChannel].size() > _analogHistoryLength)
						_analogHistory[_multiByteChannel].pop_back();
				}
				break;
			}
		}
	}
	// we have SysEx command data
	else if (_waitForData < 0){
		// we have all sysex data
		if (inputData == FIRMATA_END_SYSEX){
			_waitForData = 0;
			processSysExData(_sysExData);
			_sysExData.clear();
		}
		// still have data, collect it
		else {
			_sysExData.push_back((unsigned char)inputData);
		}
	}
	// we have a command
	else{

		int command;

		// extract the command and channel info from a byte if it is less than 0xF0
		if (inputData < 0xF0) {
			command = inputData & 0xF0;
			_multiByteChannel = inputData & 0x0F;
		}
		else {
			// commands in the 0xF* range don't use channel data
			command = inputData;
		}

		switch (command) {
		case FIRMATA_REPORT_VERSION:
		case FIRMATA_DIGITAL_MESSAGE:
		case FIRMATA_ANALOG_MESSAGE:
			_waitForData = 2;  // 2 bytes needed
			_executeMultiByteCommand = command;
			break;
		case FIRMATA_START_SYSEX:
			_sysExData.clear();
			_waitForData = -1;  // n bytes needed, -1 is used to indicate sysex message
			_executeMultiByteCommand = command;
			break;
		}

	}
}

// sysex data is assumed to be 8-bit bytes split into two 7-bit bytes.
void ofArduino::processSysExData(vector<unsigned char> data){

	string str;
	vector<unsigned char>::iterator it;
	unsigned char buffer;
	//int i = 1;
	I2C_Data i2creply;
	int stepperNumber;
	vector<Encoder_Data> encoderReply;
	Encoder_Data tempEncoderReply;
	int encoderPos = 0;
	unsigned char encBuffer[4];
<<<<<<< HEAD
=======
	unsigned char stepBuffer[4];
	Stepper_Data stepperData;
>>>>>>> origin/experimental
	// act on reserved sysEx messages (extended commands) or trigger SysEx event...
	switch (data.front()) { //first byte in buffer is command
	case FIRMATA_SYSEX_REPORT_FIRMWARE:
		it = data.begin();
		it++; // skip the first byte, which is the firmware version command
		_majorFirmwareVersion = *it;
		it++;
		_minorFirmwareVersion = *it;
		it++;

		while (it != data.end()) {
			buffer = *it;
			it++;
			if (it != data.end()){
				buffer += *it << 7;
				it++;
			}
			str += buffer;
		}
		_firmwareName = str;

		ofNotifyEvent(EFirmwareVersionReceived, _majorFirmwareVersion, this);

		// trigger the initialization event
		ofNotifyEvent(EInitialized, _majorFirmwareVersion, this);
		_initialized = true;

		break;
	case FIRMATA_SYSEX_FIRMATA_STRING:
		it = data.begin();
		it++; // skip the first byte, which is the string command
		while (it != data.end()) {
			buffer = *it;
			it++;
			if (it != data.end()) {
				buffer += *it << 7;
				it++;
			}
			str += buffer;
		}
		cout << "String Recieved: " << str << endl;
		_stringHistory.push_front(str);
		if ((int)_stringHistory.size() > _stringHistoryLength)
			_stringHistory.pop_back();


		ofNotifyEvent(EStringReceived, str, this);
		break;
	case I2C_REPLY:
		if (data.size() > 7 && (data.size() - 5) % 2 == 0){
			it = data.begin();
			it++; // skip the first byte, which is the string command

			i2creply.address = (*it & 0x7F) | ((*++it & 0x7F) << 7);
			i2creply.reg = (*++it & 0x7F) | ((*++it & 0x7F) << 7);

			while (it != data.end()) {
				buffer = *it;
				it++;
				if (it != data.end()) {
					buffer += *it << 7;
					it++;
				}
				i2creply.data += buffer;
			}
			ofNotifyEvent(EI2CDataRecieved, i2creply, this);
		}
		else {
			ofLogError("Arduino") << "Incorrect Number of Bytes recieved, possible buffer overflow";
		}
		break;
	case STEPPER_DATA:
<<<<<<< HEAD
		it = data.begin();
		it++; // skip the first byte, which is the string command
		//as is currently implemented will only ever be 1 byte long
		stepperNumber = (*it & 0x7F);

		ofNotifyEvent(EStepperIsDone, stepperNumber, this);

		break;
	case ENCODER_DATA:
		if (data.size() % 5 == 1){
			it = data.begin();
			it++; // skip the first byte, which is the string command

			while (it != data.end()) {
				tempEncoderReply.ID = (*it & ENCODER_CHANNEL_MASK);
				tempEncoderReply.direction = (*it & ENCODER_DIRECTION_MASK);

				it++;

				encBuffer[0] = *it++ & 0x7F;
				encBuffer[1] = *it++ & 0x7F;
				encBuffer[2] = *it++ & 0x7F;
				encBuffer[3] = *it++ & 0x7F;

				encoderPos = encBuffer[3];
				encoderPos <<= 7;
				encoderPos |= encBuffer[2];
				encoderPos <<= 7;
				encoderPos |= encBuffer[1];
				encoderPos <<= 7;
				encoderPos |= encBuffer[0];

				/*Firmata.write((byte)absValue & 0x7F);
				Firmata.write((byte)(absValue >> 7) & 0x7F);
				Firmata.write((byte)(absValue >> 14) & 0x7F);
				Firmata.write((byte)(absValue >> 21) & 0x7F);*/

				tempEncoderReply.position = encoderPos;
				encoderReply.push_back(tempEncoderReply);
			}
			ofNotifyEvent(EEncoderDataRecieved, encoderReply, this);
		}
		else {
			ofLogError("Arduino") << "Incorrect Number of Bytes recieved, possible buffer overflow";
		}

=======
		if (data.size() <= 8 && data.size() >= 3){
			it = data.begin();
			it++; // skip the first byte, which is the string command
			//as is currently implemented will only ever be 1 byte long
			switch (*it){
			case STEPPER_GET_POSITION:
				stepperData.type = STEPPER_GET_POSITION;
				stepperData.id = *++it;
				stepBuffer[0] = *++it & 0x7F;
				stepBuffer[1] = *++it & 0x7F;
				stepBuffer[2] = *++it & 0x7F;
				stepBuffer[3] = *++it & 0x7F;

				stepperData.data = stepBuffer[3];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[2];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[1];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[0];

				stepperData.data *= *++it ? 1 : -1;
				ofNotifyEvent(EStepperDataRecieved, stepperData, this);
				break;
			case STEPPER_GET_DISTANCE_TO:
				stepperData.type = STEPPER_GET_DISTANCE_TO;
				stepperData.id = *++it;
				stepBuffer[0] = *++it & 0x7F;
				stepBuffer[1] = *++it & 0x7F;
				stepBuffer[2] = *++it & 0x7F;
				stepBuffer[3] = *++it & 0x7F;

				stepperData.data = stepBuffer[3];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[2];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[1];
				stepperData.data <<= 7;
				stepperData.data |= stepBuffer[0];

				stepperData.data *= *++it ? 1 : -1;
				ofNotifyEvent(EStepperDataRecieved, stepperData, this);
				break;
			case STEPPER_DONE:
				stepperData.type = STEPPER_DONE;
				stepperData.id = (*++it & 0x7F);
				stepperData.data = 0;
				ofNotifyEvent(EStepperDataRecieved, stepperData, this);
				break;
			default:
				ofLogNotice("Stepper") << "Unrecognized Command Data";
				break;
			}
		}
		else {
			ofLogError("Arduino") << "Incorrect Number of Bytes recieved, possible buffer overflow";
		}
		break;
	case ENCODER_DATA:
		if (data.size() % 5 == 1){
			it = data.begin();
			it++; // skip the first byte, which is the string command

			while (it != data.end()) {
				tempEncoderReply.ID = (*it & ENCODER_CHANNEL_MASK);
				tempEncoderReply.direction = (*it & ENCODER_DIRECTION_MASK);

				it++;

				encBuffer[0] = *it++ & 0x7F;
				encBuffer[1] = *it++ & 0x7F;
				encBuffer[2] = *it++ & 0x7F;
				encBuffer[3] = *it++ & 0x7F;

				encoderPos = encBuffer[3];
				encoderPos <<= 7;
				encoderPos |= encBuffer[2];
				encoderPos <<= 7;
				encoderPos |= encBuffer[1];
				encoderPos <<= 7;
				encoderPos |= encBuffer[0];

				tempEncoderReply.position = encoderPos;
				encoderReply.push_back(tempEncoderReply);
			}
			ofNotifyEvent(EEncoderDataRecieved, encoderReply, this);
		}
		else {
			ofLogError("Arduino") << "Incorrect Number of Bytes recieved, possible buffer overflow";
		}

>>>>>>> origin/experimental
		break;
	default: // the message isn't in Firmatas extended command set
		_sysExHistory.push_front(data);
		if ((int)_sysExHistory.size() > _sysExHistoryLength)
			_sysExHistory.pop_back();
		ofNotifyEvent(ESysExReceived, data, this);
		break;

	}
}

void ofArduino::processDigitalPort(int port, unsigned char value){

	unsigned char mask;
	int previous;
	int i;
	int pin;

	for (int i = 0; i<8; ++i) {
		pin = i + (port * 8);
		if (_digitalPinMode[pin] == ARD_INPUT || _digitalPinMode[pin] == ARD_INPUT_PULLUP){
			if (!_digitalHistory[pin].empty())
				previous = _digitalHistory[pin].front();
			else previous = 0;

			mask = 1 << i;
			_digitalHistory[pin].push_front((value & mask) >> i);

			if ((int)_digitalHistory[pin].size()>_digitalHistoryLength)
				_digitalHistory[pin].pop_back();

			// trigger an event if the pin has changed value
			if (_digitalHistory[pin].front() != previous){
				ofNotifyEvent(EDigitalPinChanged, pin, this);
			}
		}
	}
}

// port 0: pins 2-7  (0,1 are serial RX/TX, don't change their values)
// port 1: pins 8-13 (14,15 are disabled for the crystal)
// port 2: pins 16-21 analog pins used as digital, all analog reporting will be turned off if this is set to ARD_ON
void ofArduino::sendDigitalPortReporting(int port, int mode){
	sendByte(FIRMATA_REPORT_DIGITAL | port);
	sendByte(mode);
	_digitalPortReporting[port] = mode;
}

void ofArduino::sendDigitalPinReporting(int pin, int mode){
	_digitalPinReporting[pin] = mode;
	if (mode == ARD_ON){	// enable reporting for the port
		sendDigitalPortReporting(pin >> 3, mode);
	}
	else if (mode == ARD_OFF) {
		bool send = true;
		int port = pin >> 3;
		for (int i = 0; i < 8; i++) {
			if (_digitalPinReporting[port * 8 + i] == ARD_ON)
				send = false;
		}
		if (send)
			sendDigitalPortReporting(port, ARD_OFF);
	}
}

void ofArduino::sendByte(unsigned char byte){
	//char msg[100];
	//sprintf(msg, "Sending Byte: %i", byte);
	//Logger::get("Application").information(msg);
	_port.writeByte(byte);
}

void ofArduino::flushAll(){
	while (_port.readByte() != -1);
	for (int i = 0; i < 5; i++)
		sendByte(FIRMATA_END_SYSEX);
}

// in Firmata (and MIDI) data bytes are 7-bits. The 8th bit serves as a flag to mark a byte as either command or data.
// therefore you need two data bytes to send 8-bits (a char).
void ofArduino::sendValueAsTwo7bitBytes(int value)
{
	sendByte(value & 0x7F); // LSB
	sendByte(value >> 7 & 127); // MSB
}

// SysEx data is sent as 8-bit bytes split into two 7-bit bytes, this function merges two 7-bit bytes back into one 8-bit byte.
int ofArduino::getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb){
	return (msb << 7) | lsb;
}

void ofArduino::sendServo(int pin, int value, bool force){
	if (_digitalPinMode[pin] == ARD_SERVO && (_servoValue[pin] != value || force)){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(SYSEX_SERVO_WRITE);
		sendByte(pin);
		sendValueAsTwo7bitBytes(value);
		sendByte(FIRMATA_END_SYSEX);
		_servoValue[pin] = value;
	}
}

void ofArduino::sendServoAttach(int pin, int minPulse, int maxPulse, int angle) {
	sendByte(FIRMATA_START_SYSEX);
	sendByte(SYSEX_SERVO_ATTACH);
	sendByte(pin);
	sendValueAsTwo7bitBytes(minPulse);
	sendValueAsTwo7bitBytes(maxPulse);
	sendByte(FIRMATA_END_SYSEX);
	_digitalPinMode[pin] = ARD_SERVO;
}

void ofArduino::sendServoDetach(int pin) {
	sendByte(FIRMATA_START_SYSEX);
	sendByte(SYSEX_SERVO_DETACH);
	sendByte(pin);
	sendByte(FIRMATA_END_SYSEX);
	_digitalPinMode[pin] = ARD_OUTPUT;
}

int ofArduino::getServo(int pin){
	if (_digitalPinMode[pin] == ARD_SERVO)
		return _servoValue[pin];
	else
		return -1;
}

<<<<<<< HEAD
void  ofArduino::sendStepper2Wire(int dirPin, int stepPin, int stepsPerRev){
=======
void  ofArduino::sendStepper2Wire(int dirPin, int stepPin, int stepsPerRev, int limitSwitch1, int limitSwitch2, bool switch1UsesPullup, bool switch2UsesPullup){
>>>>>>> origin/experimental

	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_CONFIG);
		sendByte(_stepperID);
		sendByte(DRIVER);
		sendValueAsTwo7bitBytes(stepsPerRev);
		sendByte(dirPin);
		sendByte(stepPin);
<<<<<<< HEAD
		sendByte(FIRMATA_END_SYSEX);
=======
		sendByte(0);
		sendByte(0);
		sendByte(limitSwitch1);
		sendByte(limitSwitch2);
		sendByte(switch1UsesPullup);
		sendByte(switch2UsesPullup);
		sendByte(FIRMATA_END_SYSEX);

		if (limitSwitch1 > 0)
			switch1UsesPullup ? _digitalPinMode[limitSwitch1] = ARD_INPUT_PULLUP : _digitalPinMode[limitSwitch1] = ARD_INPUT;

		if (limitSwitch2 > 0)
			switch2UsesPullup ? _digitalPinMode[limitSwitch2] = ARD_INPUT_PULLUP : _digitalPinMode[limitSwitch2] = ARD_INPUT;


>>>>>>> origin/experimental
		_digitalPinMode[dirPin] = ARD_OUTPUT;
		_digitalPinMode[stepPin] = ARD_OUTPUT;
		_stepperID++;
	}
	else {
		ofLogNotice("Arduino") << "Reached max number of steppers";
	}
}

<<<<<<< HEAD
void  ofArduino::sendStepper4Wire(int pin1, int pin2, int pin3, int pin4, int stepsPerRev){

	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_CONFIG);
		sendByte(_stepperID);
		sendByte(FOUR_WIRE);
		sendValueAsTwo7bitBytes(stepsPerRev);
		sendByte(pin1);
		sendByte(pin2);
		sendByte(pin3);
		sendByte(pin4);
		sendByte(FIRMATA_END_SYSEX);
=======
void  ofArduino::sendStepper4Wire(int pin1, int pin2, int pin3, int pin4, int stepsPerRev, int limitSwitch1, int limitSwitch2, bool switch1UsesPullup, bool switch2UsesPullup){

	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_CONFIG);
		sendByte(_stepperID);
		sendByte(FOUR_WIRE);
		sendValueAsTwo7bitBytes(stepsPerRev);
		sendByte(pin1);
		sendByte(pin2);
		sendByte(pin3);
		sendByte(pin4);
		sendByte(limitSwitch1);
		sendByte(limitSwitch2);
		sendByte(switch1UsesPullup);
		sendByte(switch2UsesPullup);
		sendByte(FIRMATA_END_SYSEX);

		if (limitSwitch1 > 0)
			switch1UsesPullup ? _digitalPinMode[limitSwitch1] = ARD_INPUT_PULLUP : _digitalPinMode[limitSwitch1] = ARD_INPUT;

		if (limitSwitch2 > 0)
			switch2UsesPullup ? _digitalPinMode[limitSwitch2] = ARD_INPUT_PULLUP : _digitalPinMode[limitSwitch2] = ARD_INPUT;


>>>>>>> origin/experimental
		_digitalPinMode[pin1] = ARD_OUTPUT;
		_digitalPinMode[pin2] = ARD_OUTPUT;
		_digitalPinMode[pin3] = ARD_OUTPUT;
		_digitalPinMode[pin4] = ARD_OUTPUT;
		_stepperID++;
	}
	else {
		ofLogNotice("Arduino") << "Reached max number of steppers";
	}

}

void ofArduino::sendStepperMove(int stepperID, int direction, int numSteps, int speed, float acceleration, float deceleration) {

	if (stepperID <= _stepperID && stepperID >= 0){
		unsigned char steps[3] = { abs(numSteps) & 0x0000007F, (abs(numSteps) >> 7) & 0x0000007F, (abs(numSteps) >> 14) & 0x0000007F };

<<<<<<< HEAD
	if (stepperID <= _stepperID && stepperID >= 0){
		unsigned char steps[3] = { abs(numSteps) & 0x0000007F, (abs(numSteps) >> 7) & 0x0000007F, (abs(numSteps) >> 14) & 0x0000007F };

=======
>>>>>>> origin/experimental
		// the stepper interface expects decimal expressed an an integer
		if (acceleration != 0 && deceleration != 0) {
			int accel = floor(acceleration * 100);
			int decel = floor(deceleration * 100);
<<<<<<< HEAD

			sendByte(FIRMATA_START_SYSEX);
			sendByte(STEPPER_DATA);
			sendByte(STEPPER_STEP);
			sendByte(stepperID);
			sendByte(direction);
			sendByte(steps[0]);
			sendByte(steps[1]);
			sendByte(steps[2]);
			sendValueAsTwo7bitBytes(speed);
			sendValueAsTwo7bitBytes(accel);
			sendValueAsTwo7bitBytes(decel);
			sendByte(FIRMATA_END_SYSEX);

		}
		else {
			sendByte(FIRMATA_START_SYSEX);
			sendByte(STEPPER_DATA);
			sendByte(STEPPER_STEP);
=======

			sendByte(FIRMATA_START_SYSEX);
			sendByte(STEPPER_DATA);
			sendByte(STEPPER_MOVE);
			sendByte(stepperID);
			sendByte(direction);
			sendByte(steps[0]);
			sendByte(steps[1]);
			sendByte(steps[2]);
			sendValueAsTwo7bitBytes(speed);
			sendValueAsTwo7bitBytes(accel);
			sendValueAsTwo7bitBytes(decel);
			sendByte(FIRMATA_END_SYSEX);

		}
		else if (speed != 0){
			sendByte(FIRMATA_START_SYSEX);
			sendByte(STEPPER_DATA);
			sendByte(STEPPER_MOVE);
>>>>>>> origin/experimental
			sendByte(stepperID);
			sendByte(direction);
			sendByte(steps[0]);
			sendByte(steps[1]);
			sendByte(steps[2]);
			sendValueAsTwo7bitBytes(speed);
			sendByte(FIRMATA_END_SYSEX);
		}
<<<<<<< HEAD
	}

}

void ofArduino::sendStepperLimitSwitch(int stepperID, int pin, bool sideOfStepper, bool usesInputPullup) {

	if (stepperID <= _stepperID && stepperID >= 0){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_LIMIT_SWITCH);
		sendByte(stepperID);
		sendByte(sideOfStepper);
		sendByte(pin);
		sendByte(usesInputPullup);
=======
		else{
			sendByte(FIRMATA_START_SYSEX);
			sendByte(STEPPER_DATA);
			sendByte(STEPPER_MOVE);
			sendByte(stepperID);
			sendByte(direction);
			sendByte(steps[0]);
			sendByte(steps[1]);
			sendByte(steps[2]);
			sendByte(FIRMATA_END_SYSEX);
		}
	}

}

void ofArduino::getStepperPosition(int stepperID){
	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_GET_POSITION);
		sendByte(stepperID);
		sendByte(FIRMATA_END_SYSEX);
	}
}

void ofArduino::getStepperDistanceFrom(int stepperID){
	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_GET_DISTANCE_TO);
		sendByte(stepperID);
		sendByte(FIRMATA_END_SYSEX);
	}
}

void ofArduino::setStepperSpeed(int stepperID, unsigned int speed){
	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_SET_SPEED);
		sendByte(stepperID);
		sendValueAsTwo7bitBytes(speed);
>>>>>>> origin/experimental
		sendByte(FIRMATA_END_SYSEX);
	}
}

<<<<<<< HEAD
		_digitalPinMode[pin] = usesInputPullup ? ARD_INPUT_PULLUP : ARD_INPUT;
		sendDigitalPinReporting(pin, ARD_ON);
	}
}

=======
void ofArduino::setStepperAcceleration(int stepperID, unsigned int accel){
	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_SET_ACCEL);
		sendByte(stepperID);
		sendValueAsTwo7bitBytes(accel);
		sendByte(FIRMATA_END_SYSEX);
	}
}

void ofArduino::setStepperDeceleration(int stepperID, unsigned int decel){
	if (_stepperID < MAX_STEPPERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(STEPPER_DATA);
		sendByte(STEPPER_SET_DECEL);
		sendByte(stepperID);
		sendValueAsTwo7bitBytes(decel);
		sendByte(FIRMATA_END_SYSEX);
	}
}

//void ofArduino::sendStepperLimitSwitch(int stepperID, int pin, bool sideOfStepper, bool usesInputPullup) {
//
//	if (stepperID <= _stepperID && stepperID >= 0){
//		sendByte(FIRMATA_START_SYSEX);
//		sendByte(STEPPER_DATA);
//		sendByte(STEPPER_LIMIT_SWITCH);
//		sendByte(stepperID);
//		sendByte(sideOfStepper);
//		sendByte(pin);
//		sendByte(usesInputPullup);
//		sendByte(FIRMATA_END_SYSEX);
//
//		_digitalPinMode[pin] = usesInputPullup ? ARD_INPUT_PULLUP : ARD_INPUT;
//		sendDigitalPinReporting(pin, ARD_ON);
//	}
//}

>>>>>>> origin/experimental
/**
* Sends a I2C config request to the arduino board with an optional
* value in microseconds to delay an I2C Read.  Must be called before
* an I2C Read or Write
* @param {number} delay in microseconds to set for I2C Read
*/

void  ofArduino::sendI2CConfig(int delay) {
	delay = delay || 0;
	sendByte(FIRMATA_START_SYSEX);
	sendByte(I2C_CONFIG);
	sendByte(delay & 0xFF);
	sendByte((delay >> 8) & 0xFF);
	sendByte(FIRMATA_END_SYSEX);

	_i2cConfigured = true;
}

/**
* Asks the arduino to send an I2C request to a device
* @param {number} slaveAddress The address of the I2C device
* @param {Array} bytes The bytes to send to the device
*/

void  ofArduino::sendI2CWriteRequest(char slaveAddress, unsigned char * bytes, int numOfBytes) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(slaveAddress);
		sendByte(WRITE << 3);

		for (int i = 0, length = numOfBytes; i < length; i++) {
			sendValueAsTwo7bitBytes(bytes[i]);
		}

		sendByte(FIRMATA_END_SYSEX);
	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}

void  ofArduino::sendI2CWriteRequest(char slaveAddress, vector<char> bytes) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(slaveAddress);
		sendByte(WRITE << 3);

		for (int i = 0, length = bytes.size(); i < length; i++) {
			sendValueAsTwo7bitBytes(bytes[i]);
		}

		sendByte(FIRMATA_END_SYSEX);
	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}
/**
* Write data to a register
*
* @param {number} address      The address of the I2C device.
* @param {array} cmdRegOrData  An array of bytes
*
* Write a command to a register
*
* @param {number} address      The address of the I2C device.
* @param {number} cmdRegOrData The register
* @param {array} inBytes       An array of bytes
*
*/
void  ofArduino::i2cWrite(char address, unsigned char * bytes, int numOfBytes) {
	/**
	* registerOrData:
	* [... arbitrary bytes]
	*
	* or
	*
	* registerOrData, inBytes:
	* command [, ...]
	*
	*/
	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(address);
		sendByte(WRITE << 3);

		for (int i = 0, length = numOfBytes; i < length; i++) {
			sendValueAsTwo7bitBytes(bytes[i]);
		}

		sendByte(FIRMATA_END_SYSEX);
	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}

/**
* Write data to a register
*
* @param {number} address    The address of the I2C device.
* @param {number} register   The register.
* @param {number} byte       The byte value to write.
*
*/

void  ofArduino::i2cWriteReg(char address, int reg, int byte) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(address);
		sendByte(WRITE << 3);
		sendValueAsTwo7bitBytes(reg);
		sendValueAsTwo7bitBytes(byte);
		sendByte(FIRMATA_END_SYSEX);
	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}


/**
* Asks the arduino to request bytes from an I2C device
* @param {number} slaveAddress The address of the I2C device
* @param {number} numBytes The number of bytes to receive.
* @param {function} callback A function to call when we have received the bytes.
*/

void  ofArduino::sendI2CReadRequest(char address, unsigned char numBytes) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(address);
		sendByte(READ << 3);
		sendValueAsTwo7bitBytes(numBytes);
		sendByte(FIRMATA_END_SYSEX);
	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}

// TODO: Refactor i2cRead and i2cReadOnce
//      to share most operations.

/**
* Initialize a continuous I2C read.
*
* @param {number} address    The address of the I2C device
* @param {number} register   Optionally set the register to read from.
* @param {number} numBytes   The number of bytes to receive.
*
*/

void  ofArduino::i2cRead(char address, unsigned char reg, int bytesToRead) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(address);
		sendByte(CONTINUOUS_READ << 3);
		sendValueAsTwo7bitBytes(reg);
		sendValueAsTwo7bitBytes(bytesToRead);
		sendByte(FIRMATA_END_SYSEX);

	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}

/**
* Perform a single I2C read
*
* Supersedes sendI2CReadRequest
*
* Read bytes from address
*
* @param {number} address    The address of the I2C device
* @param {number} register   Optionally set the register to read from.
* @param {number} numBytes   The number of bytes to receive.
*
*/


void  ofArduino::i2cReadOnce(char address, unsigned char reg, int bytesToRead) {

	if (_i2cConfigured){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(I2C_REQUEST);
		sendByte(address);
		sendByte(READ << 3);
		sendValueAsTwo7bitBytes(reg);
		sendValueAsTwo7bitBytes(bytesToRead);
		sendByte(FIRMATA_END_SYSEX);

	}
	else {
		ofLogNotice("Arduino") << "I2C was not configured, did you send an I2C config request?";
	}
}

bool ofArduino::isI2CConfigured() {
	return  _i2cConfigured;
}

// CONTINUOUS_READ

///**
//* Configure the passed pin as the controller in a 1-wire bus.
//* Pass as enableParasiticPower true if you want the data pin to power the bus.
//* @param pin
//* @param enableParasiticPower
//*/
void  ofArduino::sendOneWireConfig(int pin, bool enableParasiticPower) {
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ONEWIRE_DATA);
	sendByte(ONEWIRE_CONFIG_REQUEST);
	sendByte(pin);
	sendByte(enableParasiticPower ? 0x01 : 0x00);
	sendByte(FIRMATA_END_SYSEX);
};

///**
//* Searches for 1-wire devices on the bus.  The passed callback should accept
//* and error argument and an array of device identifiers.
//* @param pin
//* @param callback
//*/
void  ofArduino::sendOneWireSearch(int pin) {
	sendOneWireSearch(ONEWIRE_SEARCH_REQUEST, pin);
};

///**
//* Searches for 1-wire devices on the bus in an alarmed state.  The passed callback
//* should accept and error argument and an array of device identifiers.
//* @param pin
//* @param callback
//*/
void  ofArduino::sendOneWireAlarmsSearch(int pin) {
	sendOneWireSearch(ONEWIRE_SEARCH_ALARMS_REQUEST, pin);
};


//needs to notify event handler
void  ofArduino::sendOneWireSearch(char type, int pin) {
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ONEWIRE_DATA);
	sendByte(type);
	sendByte(pin);
	sendByte(FIRMATA_END_SYSEX);
}

///**
//* Reads data from a device on the bus and invokes the passed callback.
//*
//* N.b. ConfigurableFirmata will issue the 1-wire select command internally.
//* @param pin
//* @param device
//* @param numBytesToRead
//* @param callback
//*/
void  ofArduino::sendOneWireRead(int pin, unsigned char device, int numBytesToRead) {
	int correlationId = floor(ofRandomuf() * 255);
	sendOneWireRequest(pin, ONEWIRE_READ_REQUEST_BIT, device, numBytesToRead, correlationId, 0, 0);
}

///**
//* Resets all devices on the bus.
//* @param pin
//*/
void  ofArduino::sendOneWireReset(int pin) {
	sendOneWireRequest(pin, ONEWIRE_RESET_REQUEST_BIT, 0, 0, 0, 0, 0);
};

///**
//* Writes data to the bus to be received by the passed device.  The device
//* should be obtained from a previous call to sendOneWireSearch.
//*
//* N.b. ConfigurableFirmata will issue the 1-wire select command internally.
//* @param pin
//* @param device
//* @param data
//*/
void  ofArduino::sendOneWireWrite(int pin, unsigned char device, unsigned char * data) {
	sendOneWireRequest(pin, ONEWIRE_WRITE_REQUEST_BIT, device, 0, 0, 0, data);
};

///**
//* Tells firmata to not do anything for the passed amount of ms.  For when you
//* need to give a device attached to the bus time to do a calculation.
//* @param pin
//*/
void  ofArduino::sendOneWireDelay(int pin, int delay) {
	sendOneWireRequest(pin, ONEWIRE_DELAY_REQUEST_BIT, 0, 0, 0, delay, 0);
};

///**
//* Sends the passed data to the passed device on the bus, reads the specified
//* number of bytes and invokes the passed callback.
//*
//* N.b. ConfigurableFirmata will issue the 1-wire select command internally.
//* @param pin
//* @param device
//* @param data
//* @param numBytesToRead
//* @param callback
//*/
void  ofArduino::sendOneWireWriteAndRead(int pin, unsigned char device, unsigned char * data, int numBytesToRead) {
	int correlationId = floor(ofRandomuf() * 255);
	sendOneWireRequest(pin, ONEWIRE_WRITE_REQUEST_BIT | ONEWIRE_READ_REQUEST_BIT, device, numBytesToRead, correlationId, 0, data);
}

//// see http://firmata.org/wiki/Proposals#OneWire_Proposal
void  ofArduino::sendOneWireRequest(int pin, unsigned char subcommand, unsigned char device, int numBytesToRead, unsigned char correlationId, int delay, unsigned char * dataToWrite) {
<<<<<<< HEAD
}
//	unsigned char bytes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//
//	if (device || numBytesToRead || correlationId || delay || dataToWrite) {
//		subcommand = subcommand | ONEWIRE_WITHDATA_REQUEST_BITS;
//	}
//
//	if (device) {
//		bytes.splice.apply(bytes, [0, 8].concat(device));
//	}
//
//	if (numBytesToRead) {
//		bytes[8] = numBytesToRead & 0xFF;
//		bytes[9] = (numBytesToRead >> 8) & 0xFF;
//	}
//
//	if (correlationId) {
//		bytes[10] = correlationId & 0xFF;
//		bytes[11] = (correlationId >> 8) & 0xFF;
//	} 
//
//	if (delay) {
//		bytes[12] = delay & 0xFF;
//		bytes[13] = (delay >> 8) & 0xFF;
//		bytes[14] = (delay >> 16) & 0xFF;
//		bytes[15] = (delay >> 24) & 0xFF;
//	}
//
//	if (dataToWrite) {
//		dataToWrite.forEach(function(byte) {
//			bytes.push(byte);
//		});
//	}
//
//	var output = [START_SYSEX, ONEWIRE_DATA, subcommand, pin];
//	output = output.concat(Encoder7Bit.to7BitArray(bytes));
//	output.push(END_SYSEX);
//
//	sendByte(FIRMATA_START_SYSEX);
//	sendByte(ONEWIRE_DATA);
//	sendByte(subcommand);
//	sendByte(pin);
//	sendByte(FIRMATA_END_SYSEX);
//}

void ofArduino::attachEncoder(int pinA, int pinB){
	if (_encoderID < MAX_ENCODERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(ENCODER_DATA);
		sendByte(ENCODER_ATTACH);
		sendByte(_encoderID);
		sendByte(pinA);
		sendByte(pinB);
		sendByte(FIRMATA_END_SYSEX);
		_encoderID++;
	}

}
=======

	unsigned char bytes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	if (device || numBytesToRead || correlationId || delay || dataToWrite) {
		subcommand = subcommand | ONEWIRE_WITHDATA_REQUEST_BITS;
	}
	//
	//	if (device) {
	//		bytes.splice.apply(bytes, [0, 8].concat(device));
	//	}
	//
	if (numBytesToRead) {
		bytes[8] = numBytesToRead & 0xFF;
		bytes[9] = (numBytesToRead >> 8) & 0xFF;
	}

	if (correlationId) {
		bytes[10] = correlationId & 0xFF;
		bytes[11] = (correlationId >> 8) & 0xFF;
	}

	if (delay) {
		bytes[12] = delay & 0xFF;
		bytes[13] = (delay >> 8) & 0xFF;
		bytes[14] = (delay >> 16) & 0xFF;
		bytes[15] = (delay >> 24) & 0xFF;
	}

	//if (dataToWrite) {
	//	dataToWrite.forEach(function(byte) {
	//		bytes.push(byte);
	//	});
	//}
	//
	//	var output = [START_SYSEX, ONEWIRE_DATA, subcommand, pin];
	//	output = output.concat(Encoder7Bit.to7BitArray(bytes));
	//	output.push(END_SYSEX);
	//
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ONEWIRE_DATA);
	sendByte(subcommand);
	sendByte(pin);
	sendByte(FIRMATA_END_SYSEX);
}

void ofArduino::attachEncoder(int pinA, int pinB){
	if (_encoderID < MAX_ENCODERS){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(ENCODER_DATA);
		sendByte(ENCODER_ATTACH);
		sendByte(_encoderID);
		sendByte(pinA);
		sendByte(pinB);
		sendByte(FIRMATA_END_SYSEX);
		_encoderID++;
	}

}
//void ofArduino::attachEncoder(int encoderID, int pinA, int pinB){
//	if (encoderID < MAX_ENCODERS){
//		sendByte(FIRMATA_START_SYSEX);
//		sendByte(ENCODER_DATA);
//		sendByte(ENCODER_ATTACH);
//		sendByte(encoderID);
//		sendByte(pinA);
//		sendByte(pinB);
//		sendByte(FIRMATA_END_SYSEX);
//	}
//
//}
>>>>>>> origin/experimental
void ofArduino::getEncoderPosition(int encoderID){
	if (encoderID <= _encoderID && encoderID >= 0){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(ENCODER_DATA);
		sendByte(ENCODER_REPORT_POSITION);
		sendByte(encoderID);
		sendByte(FIRMATA_END_SYSEX);
	}
}
void ofArduino::getAllEncoderPositions(){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ENCODER_DATA);
	sendByte(ENCODER_REPORT_POSITIONS);
	sendByte(FIRMATA_END_SYSEX);
}
void ofArduino::resetEncoderPosition(int encoderID){
	if (encoderID <= _encoderID && encoderID >= 0){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(ENCODER_DATA);
		sendByte(ENCODER_RESET_POSITION);
		sendByte(encoderID);
		sendByte(FIRMATA_END_SYSEX);
	}
}
void ofArduino::enableEncoderReporting(){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ENCODER_DATA);
	sendByte(ENCODER_REPORT_AUTO);
	sendByte(1);
	sendByte(FIRMATA_END_SYSEX);
}
void ofArduino::disableEncoderReporting(){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(ENCODER_DATA);
	sendByte(ENCODER_REPORT_AUTO);
	sendByte(0);
	sendByte(FIRMATA_END_SYSEX);
}
void ofArduino::detachEncoder(int encoderID){
	if (encoderID <= _encoderID && encoderID >= 0){
		sendByte(FIRMATA_START_SYSEX);
		sendByte(ENCODER_DATA);
		sendByte(ENCODER_DETACH);
		sendByte(encoderID);
		sendByte(FIRMATA_END_SYSEX);
		_encoderID--;
	}
}