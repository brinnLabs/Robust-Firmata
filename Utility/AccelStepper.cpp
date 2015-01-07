// AccelStepper.cpp
//
// Copyright (C) 2009 Mike McCauley
// $Id: AccelStepper.cpp,v 1.14 2012/12/22 21:41:22 mikem Exp mikem $

#include "AccelStepper.h"

#if 0
// Some debugging assistance
void dump(byte* p, int l)
{
	int i;

	for (i = 0; i < l; i++)
	{
		Serial.print(p[i], HEX);
		Serial.print(" ");
	}
	Serial.println("");
}
#endif

AccelStepper::AccelStepper(){}

AccelStepper::AccelStepper(byte interface, byte pin1, byte pin2, byte pin3, byte pin4)
{
	_interface = interface;
	_currentPos = 0;
	_targetPos = 0;
	_speed = 0;
	_maxSpeed = 0;
	_acceleration = 1;
	_deceleration = 1;
	_stepInterval = 0;
	_minPulseWidth = 1;
	_enablePin = 0xff;
	_lastStepTime = 0;
	_pin[0] = pin1;
	_pin[1] = pin2;
	_pin[2] = pin3;
	_pin[3] = pin4;

	// NEW
	_stepCounter = 0;
	_initialStepSize = 0.0;
	_prevStepSize = 0.0;
	_minStepSize = 1.0;
	_direction = DIRECTION_CCW;

	int i;
	for (i = 0; i < 4; i++)
		_pinInverted[i] = 0;
	enableOutputs();
}

AccelStepper::AccelStepper(void(*forward)(), void(*backward)())
{
	_interface = 0;
	_currentPos = 0;
	_targetPos = 0;
	_speed = 0;
	_maxSpeed = 1;
	_acceleration = 1;
	_deceleration = 1;
	_stepInterval = 0;
	_minPulseWidth = 1;
	_enablePin = 0xff;
	_lastStepTime = 0;
	_pin[0] = 0;
	_pin[1] = 0;
	_pin[2] = 0;
	_pin[3] = 0;
	_forward = forward;
	_backward = backward;

	// NEW
	_stepCounter = 0;
	_initialStepSize = 0.0;
	_prevStepSize = 0.0;
	_minStepSize = 1.0;
	_direction = DIRECTION_CCW;

	int i;
	for (i = 0; i < 4; i++)
		_pinInverted[i] = 0;
}

void AccelStepper::moveTo(int16_t absolute)
{
	if (_targetPos != absolute)
	{
		_targetPos = absolute;
		computeNewSpeed();
		// compute new n?
	}
}

void AccelStepper::move(int16_t relative)
{
	moveTo(_currentPos + relative);
}

int16_t AccelStepper::distanceToGo()
{
	return _targetPos - _currentPos;
}

int16_t AccelStepper::targetPosition()
{
	return _targetPos;
}

int16_t AccelStepper::currentPosition()
{
	return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(int16_t position)
{
	_targetPos = _currentPos = position;
	_stepCounter = 0;
	_stepInterval = 0;
}

void AccelStepper::computeNewSpeed()
{
	int16_t distanceTo = distanceToGo(); // +ve is clockwise from curent location

	int16_t stepsToStop = (int16_t)((_speed * _speed) / (2 *_acceleration)); // Equation 16

	if (distanceTo == 0 && stepsToStop <= 1)
	{
		// We are at the target and its time to stop
		_stepInterval = 0;
		_speed = 0;
		_stepCounter = 0;
		return;
	}

	if (distanceTo > 0)
	{
		// We are anticlockwise from the target
		// Need to go clockwise from here, maybe decelerate now
		if (_stepCounter > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
				_stepCounter = -stepsToStop; // Start deceleration
		}
		else if (_stepCounter < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
				_stepCounter = -_stepCounter; // Start accceleration
		}
	}
	else if (distanceTo < 0)
	{
		// We are clockwise from the target
		// Need to go counterclockwise from here, maybe decelerate
		if (_stepCounter > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
				_stepCounter = -stepsToStop; // Start deceleration
		}
		else if (_stepCounter < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
				_stepCounter = -_stepCounter; // Start accceleration
		}
	}

	// Need to accelerate or decelerate
	if (_stepCounter == 0)
	{
		// First step from stopped
		_prevStepSize = _initialStepSize;
		_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	else
	{
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		_prevStepSize = _prevStepSize - ((2.0 * _prevStepSize) / ((4.0 * _stepCounter) + 1)); // Equation 13
		_prevStepSize = max(_prevStepSize, _minStepSize);
	}
	_stepCounter++;
	_stepInterval = _prevStepSize;
	_speed = 1000000.0 / _prevStepSize;
	if (_direction == DIRECTION_CCW)
		_speed = -_speed;

#if 0
	Serial.println(_speed);
	Serial.println(_acceleration);
	Serial.println(_deceleration);
	Serial.println(_prevStepSize);
	Serial.println(_initialStepSize);
	Serial.println(_stepCounter);
	Serial.println(_stepInterval);
	Serial.println(distanceTo);
	Serial.println(stepsToStop);
	Serial.println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
boolean AccelStepper::run()
{
if(_speed != 0 || distanceToGo() != 0)
return false;
	if (runSpeed())
		computeNewSpeed();
	return true;
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper::runSpeed()
{
	// Dont do anything unless we actually have a step interval
	if (!_stepInterval)
		return false;

	unsigned long time = micros();
	// Gymnastics to detect wrapping of either the nextStepTime and/or the current time
	unsigned long nextStepTime = _lastStepTime + _stepInterval;
	if (((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime)))
		|| ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))

	{
		if (_direction == DIRECTION_CW)
		{
			// Clockwise
			_currentPos += 1;
		}
		else
		{
			// Anticlockwise  
			_currentPos -= 1;
		}
		step(_currentPos & 0x7); // Bottom 3 bits (same as mod 8, but works with + and - numbers) 

		_lastStepTime = time;
		return true;
	}
	else
	{
		return false;
	}
}

// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
	while (_speed != 0 || distanceToGo() != 0)
		run();
}

boolean AccelStepper::runSpeedToPosition()
{
	if (_targetPos == _currentPos)
		return true;
	if (_targetPos > _currentPos)
		_direction = DIRECTION_CW;
	else
		_direction = DIRECTION_CCW;

	runSpeed();
	return false;
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(int16_t position)
{
	moveTo(position);
	runToPosition();
}

void AccelStepper::setMaxSpeed(uint16_t speed)
{
	if (_maxSpeed != speed)
	{
		_maxSpeed = speed;
		_minStepSize = 1000000.0 / speed;
		// Recompute _stepCounter from current speed and adjust speed if accelerating or cruising
		if (_stepCounter > 0)
		{
			_stepCounter = (int16_t)((_speed * _speed) / (2 * _acceleration)); // Equation 16
			computeNewSpeed();
		}
	}
}

void AccelStepper::setAcceleration(uint16_t acceleration)
{
	if (acceleration == 0)
		return;
	if (_acceleration != acceleration)
	{
		// Recompute _stepCounter per Equation 17
		_stepCounter = _stepCounter * (_acceleration / acceleration);
		// New c0 per Equation 7
		_initialStepSize = sqrt(2.0 / acceleration) * 1000000.0;
		_acceleration = acceleration;
		computeNewSpeed();
	}
}

void AccelStepper::setDeceleration(uint16_t deceleration)
{
	if (deceleration == 0)
		return;
	if (_deceleration != deceleration)
	{
		// Recompute _stepCounter per Equation 17
		_stepCounter = _stepCounter * (_deceleration / deceleration);
		_deceleration = deceleration;
		computeNewSpeed();
	}
}

void AccelStepper::setSpeed(uint16_t speed)
{
	if (speed == _speed)
		return;
	speed = constrain(speed, 0, _maxSpeed);
	if (speed == 0.0)
		_stepInterval = 0;
	else
	{
		_stepInterval = 1000000.0 / speed;
	}
	_speed = speed;
}

uint16_t AccelStepper::speed()
{
	return _speed;
}

// Subclasses can override
void AccelStepper::step(byte step)
{
	switch (_interface)
	{
	case FUNCTION:
		step0(step);
		break;

	case DRIVER:
		step1(step);
		break;

	case FULL2WIRE:
		step2(step);
		break;

	case FULL3WIRE:
		step3(step);
		break;

	case FULL4WIRE:
		step4(step);
		break;

	case HALF3WIRE:
		step6(step);
		break;

	case HALF4WIRE:
		step8(step);
		break;
	}
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(byte mask)
{
	byte numpins = 2;
	if (_interface == FULL4WIRE || _interface == HALF4WIRE)
		numpins = 4;
	byte i;
	for (i = 0; i < numpins; i++)
		digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step0(byte step)
{
	if (_speed > 0)
		_forward();
	else
		_backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step1(byte step)
{
	// _pin[0] is step, _pin[1] is direction
	setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
	// Caution 200ns setup time 
	// Delay the minimum allowed pulse width
	delayMicroseconds(_minPulseWidth);
	setOutputPins(_direction ? 0b10 : 0b00); // step LOW

}

// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step2(byte step)
{
	switch (step & 0x3)
	{
	case 0: /* 01 */
		setOutputPins(0b10);
		break;

	case 1: /* 11 */
		setOutputPins(0b11);
		break;

	case 2: /* 10 */
		setOutputPins(0b01);
		break;

	case 3: /* 00 */
		setOutputPins(0b00);
		break;
	}
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step3(byte step)
{
	switch (step % 3)
	{
	case 0:    // 100
		setOutputPins(0b100);
		break;

	case 1:    // 001
		setOutputPins(0b001);
		break;

	case 2:    //010
		setOutputPins(0b010);
		break;

	}
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step4(byte step)
{
	switch (step & 0x3)
	{
	case 0:    // 1010
		setOutputPins(0b0101);
		break;

	case 1:    // 0110
		setOutputPins(0b0110);
		break;

	case 2:    //0101
		setOutputPins(0b1010);
		break;

	case 3:    //1001
		setOutputPins(0b1001);
		break;
	}
}

// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step6(byte step)
{
	switch (step % 6)
	{
	case 0:    // 100
		setOutputPins(0b100);
		break;

	case 1:    // 101
		setOutputPins(0b101);
		break;

	case 2:    // 001
		setOutputPins(0b001);
		break;

	case 3:    // 011
		setOutputPins(0b011);
		break;

	case 4:    // 010
		setOutputPins(0b010);
		break;

	case 5:    //011
		setOutputPins(0b110);
		break;

	}
}

// 4 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step8(byte step)
{
	switch (step & 0x7)
	{
	case 0:    // 1000
		setOutputPins(0b0001);
		break;

	case 1:    // 1010
		setOutputPins(0b0101);
		break;

	case 2:    // 0010
		setOutputPins(0b0100);
		break;

	case 3:    // 0110
		setOutputPins(0b0110);
		break;

	case 4:    // 0100
		setOutputPins(0b0010);
		break;

	case 5:    //0101
		setOutputPins(0b1010);
		break;

	case 6:    // 0001
		setOutputPins(0b1000);
		break;

	case 7:    //1001
		setOutputPins(0b1001);
		break;
	}
}

// Prevents power consumption on the outputs
void AccelStepper::disableOutputs()
{
	if (!_interface) return;

	setOutputPins(0); // Handles inversion automatically
	if (_enablePin != 0xff)
		digitalWrite(_enablePin, LOW ^ _enableInverted);
}

void AccelStepper::enableOutputs()
{
	if (!_interface)
		return;

	pinMode(_pin[0], OUTPUT);
	pinMode(_pin[1], OUTPUT);
	if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	{
		pinMode(_pin[2], OUTPUT);
		pinMode(_pin[3], OUTPUT);
	}

	if (_enablePin != 0xff)
	{
		digitalWrite(_enablePin, HIGH ^ _enableInverted);
		pinMode(_enablePin, OUTPUT);
	}
}

void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
	_minPulseWidth = minWidth;
}

void AccelStepper::setEnablePin(byte enablePin)
{
	_enablePin = enablePin;

	// This happens after construction, so init pin now.
	if (_enablePin != 0xff)
	{
		digitalWrite(_enablePin, HIGH ^ _enableInverted);
		pinMode(_enablePin, OUTPUT);
	}
}

void AccelStepper::setPinsInverted(bool direction, bool step, bool enable)
{
	_pinInverted[0] = step;
	_pinInverted[1] = direction;
	_enableInverted = enable;
}

void AccelStepper::stop()
{
	if (_speed != 0)
	{
		int16_t stepsToStop = (int16_t)((_speed * _speed) / (2 * _acceleration)) + 1; // Equation 16 (+integer rounding)
		if (_speed > 0)
			move(stepsToStop);
		else
			move(-stepsToStop);
	}
}
