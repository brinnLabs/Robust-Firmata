/*
 * This is a simple example use of ofArduino
 *
 * ofArduino currently only supports the standard Arduino boards
 * (UNO, Duemilanove, Diecimila, NG, and other boards based on the
 * ATMega168 or ATMega328 microcontrollers
 * The Arduio FIO and Arduino Mini should also work.
 * The Arduino MEGA and other variants based on microcontrollers
 * other than the ATMega168 and ATMega328 are not currently supported.
 *
 * To use this example, open Arduino (preferably Arduino 1.0) and
 * navigate to File -> Examples -> Firmata and open StandardFirmata.
 * Compile and upload StandardFirmata for your board, then close
 * the Arduino application and run this application.
 *
 * If you have a servo attached, press the left arrow key to rotate
 * the servo head counterclockwise and press the right arrow key to
 * rotate the servo head clockwise.
 *
 * Clicking the mouse over any part of the application will turn the
 * on-board LED on and off.
 *
 */

#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup(){

	/*ofSetVerticalSync(true);
	ofSetFrameRate(60);*/

	ofBackground(255, 0, 130);

	buttonState = "digital pin:";
	potValue = "analog pin:";

	bgImage.loadImage("background.png");
	font.loadFont("franklinGothic.otf", 20);
	smallFont.loadFont("franklinGothic.otf", 14);

	// replace the string below with the serial port for your Arduino board
	// you can get this from the Arduino application or via command line
	// for OSX, in your terminal type "ls /dev/tty.*" to get a list of serial devices

	

#ifdef TARGET_WIN32
	//ard.connect("COM4", 57600);
	vector<string> devices;
	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	vector<string> temp = serial.getDeviceFriendlyNames();
	for (int i = 0; i < temp.size(); i++){
<<<<<<< HEAD
		//teensy's friendly name is USB Serial max speed = 125000
=======
		//we can customize this so that we don't have change ports
		//it makes the code far more portable 
		//teensy's friendly name is USB Serial max speed = 125000 
>>>>>>> origin/experimental
		if (strstr(temp[i].c_str(), "Arduino") != NULL || strstr(temp[i].c_str(), "Serial") != NULL){
			cout << "Setting up firmata on " << deviceList[i].getDeviceName() << endl;
			arduinoAttached = ard.connect(deviceList[i].getDevicePath(), 57600); 
		}
	}
#else
	ard.connect("/dev/tty.usbmodemfd121", 57600);
#endif
	// listen for EInitialized notification. this indicates that
	// the arduino is ready to receive commands and it is safe to
	// call setupArduino()
	ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
	bSetupArduino = false;	// flag so we setup arduino when its ready, you don't need to touch this :)
}

//--------------------------------------------------------------
void ofApp::update(){
	if (arduinoAttached){
		updateArduino();
		if (!bSetupArduino){
			if (ard.isArduinoReady()){
				ofLogNotice("Connecting") << "setting up arduino";
				setupArduino(0);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::setupArduino(const int & version) {
	// remove listener because we don't need it anymore
	ofRemoveListener(ard.EInitialized, this, &ofApp::setupArduino);

	// it is now safe to send commands to the Arduino
	bSetupArduino = true;

	// print firmware name and version to the console
	ofLogNotice() << ard.getFirmwareName();
	ofLogNotice() << "firmata v" << ard.getMajorFirmwareVersion() << "." << ard.getMinorFirmwareVersion();

	// Note: pins A0 - A5 can be used as digital input and output.
	// Refer to them as pins 14 - 19 if using StandardFirmata from Arduino 1.0.
	// If using Arduino 0022 or older, then use 16 - 21.
	// Firmata pin numbering changed in version 2.3 (which is included in Arduino 1.0)

	// set pins D2 and A5 to digital input
	ard.sendDigitalPinMode(2, ARD_INPUT);
<<<<<<< HEAD
	ard.sendDigitalPinMode(19, ARD_INPUT_PULLUP);  // pin 21 if using StandardFirmata from Arduino 0022 or older
=======
	ard.sendDigitalPinMode(19, ARD_INPUT_PULLUP);  
>>>>>>> origin/experimental

	// set pin A0 to analog input
	ard.sendAnalogPinReporting(0, ARD_ANALOG);

	// set pin D13 as digital output
	ard.sendDigitalPinMode(13, ARD_OUTPUT);
	// set pin A4 as digital output
<<<<<<< HEAD
	ard.sendDigitalPinMode(18, ARD_OUTPUT);  // pin 20 if using StandardFirmata from Arduino 0022 or older
=======
	ard.sendDigitalPinMode(18, ARD_OUTPUT);  
>>>>>>> origin/experimental

	// set pin D11 as PWM (analog output)
	ard.sendDigitalPinMode(11, ARD_PWM);

	// attach a servo to pin D9
	ard.sendServoAttach(9);

<<<<<<< HEAD
	// attach a stepper motor, we need to give it an id which starts at 0
	// also need a step and a dir pin for driver boards or 2 wire steppers
	// optional addition of the steps per revolution, usually 200 at 1.8 degrees per step
	ard.sendStepper2Wire(8, 7, 200);
=======
	// attach a stepper motor, we need a step and a dir pin for driver boards or 2 wire steppers
	// optional addition of the steps per revolution, usually 200 at 1.8 degrees per step
	// also optional are limit switches, the pins and whether to use input pullup or not
	ard.sendStepper2Wire(8, 7 , 3200, 3, 4);
>>>>>>> origin/experimental

	//4 wire stepper also can be sent
	//ard.sendStepper4Wire(0, 5, 6, 7, 8);

<<<<<<< HEAD
	//We probably need limit switches for our motors, though are totally optional
	//send the id of the stepper, the pin of the switch
	//the side of the stepper and whether to use input_pullup
	ard.sendStepperLimitSwitch(0, 4, true, true);

=======
>>>>>>> origin/experimental
	ard.attachEncoder(5, 6);

	ard.enableEncoderReporting();

	// Listen for changes on the digital and analog pins
	ofAddListener(ard.EDigitalPinChanged, this, &ofApp::digitalPinChanged);
	ofAddListener(ard.EAnalogPinChanged, this, &ofApp::analogPinChanged);
	ofAddListener(ard.EEncoderDataRecieved, this, &ofApp::encoderDataRecieved);
<<<<<<< HEAD
	ofAddListener(ard.EStepperIsDone, this, &ofApp::stepperFinished);
=======
	ofAddListener(ard.EStepperDataRecieved, this, &ofApp::stepperDataRecieved);
>>>>>>> origin/experimental
}

//--------------------------------------------------------------
void ofApp::updateArduino(){

	// update the arduino, get any data or messages.
	// the call to ard.update() is required
	ard.update();

	// do not send anything until the arduino has been set up
	if (bSetupArduino) {
		// fade the led connected to pin D11
		ard.sendPwm(11, (int)(128 + 128 * sin(ofGetElapsedTimef())));   // pwm...
	}

}

// digital pin event handler, called whenever a digital pin value has changed
// note: if an analog pin has been set as a digital pin, it will be handled
// by the digitalPinChanged function rather than the analogPinChanged function.

//--------------------------------------------------------------
void ofApp::digitalPinChanged(const int & pinNum) {
	// do something with the digital input. here we're simply going to print the pin number and
	// value to the screen each time it changes
	buttonState = "digital pin: " + ofToString(pinNum) + " = " + ofToString(ard.getDigital(pinNum));
}

// analog pin event handler, called whenever an analog pin value has changed

//--------------------------------------------------------------
void ofApp::analogPinChanged(const int & pinNum) {
	// do something with the analog input. here we're simply going to print the pin number and
	// value to the screen each time it changes
	potValue = "analog pin: " + ofToString(pinNum) + " = " + ofToString(ard.getAnalog(pinNum));
}

void ofApp::encoderDataRecieved(const vector<Encoder_Data> & data){
	cout << "recieved " + ofToString(data.size()) +" encoder data" <<  endl;
	for (int i = 0; i < data.size(); i++){
		cout << "ID: " + ofToString(data[i].ID) + " Direction: " + ofToString(data[i].direction) + " Position: " + ofToString(data[i].position) << endl;
	}
	
}
<<<<<<< HEAD
void ofApp::stepperFinished(const int & stepperID){
	cout << "recieved stepper data" << endl;
=======
void ofApp::stepperDataRecieved(const Stepper_Data & data){
	switch (data.type){
	case STEPPER_GET_POSITION:
		cout << "Stepper " << data.id << "'s current position: " + ofToString(data.data) << endl;
		break;
	case STEPPER_GET_DISTANCE_TO:
		cout << "Stepper " << data.id << "'s distance from target position: " + ofToString(data.data) << endl;
		break;
	case STEPPER_DONE:
		cout << "Stepper " << data.id << "' is done stepping" << endl;
		break;
	default:
		break;
	}
	
>>>>>>> origin/experimental
}


//--------------------------------------------------------------
void ofApp::draw(){
	bgImage.draw(0, 0);

	ofEnableAlphaBlending();
	ofSetColor(0, 0, 0, 127);
	ofRect(510, 15, 275, 150);
	ofDisableAlphaBlending();

	ofSetColor(255, 255, 255);
	if (!bSetupArduino){
		font.drawString("arduino not ready...\n", 515, 40);
	}
	else {
<<<<<<< HEAD
		/*font.drawString(potValue + "\n" + buttonState +
			"\nsending pwm: " + ofToString((int)(128 + 128 * sin(ofGetElapsedTimef()))), 515, 40);*/
=======
		font.drawString(potValue + "\n" + buttonState +
			"\nsending pwm: " + ofToString((int)(128 + 128 * sin(ofGetElapsedTimef()))), 515, 40);
>>>>>>> origin/experimental

		ofSetColor(64, 64, 64);
		smallFont.drawString("If a servo is attached, use the left arrow key to rotate "
			"\ncounterclockwise and the right arrow key to rotate clockwise.", 200, 550);
		ofSetColor(255, 255, 255);

	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
	case OF_KEY_UP:
<<<<<<< HEAD
		// turn the stepper 200 steps
		ard.sendStepperStep(0, CCW, 1000, 10000, 20, 20);
		break;
	case OF_KEY_DOWN:
		// turn it the opposite direction
		ard.sendStepperStep(0, CW, 200, 255);
=======
		// turn the stepper one revolution
		// can set speed and acceleration and deceleration
		//ard.sendStepperMove(0, CCW, 3200, 10000, 20, 20);
		ard.sendStepperMove(0, CCW, 3200);
		break;
	case OF_KEY_DOWN:
		// turn it the opposite direction
		ard.sendStepperMove(0, CW, 3200);
>>>>>>> origin/experimental
		break;
	case OF_KEY_RIGHT:
		// rotate servo head to 180 degrees
		ard.sendServo(9, 180, false);
		ard.sendDigital(18, ARD_HIGH);  // pin 20 if using StandardFirmata from Arduino 0022 or older
		break;
	case OF_KEY_LEFT:
		// rotate servo head to 0 degrees
		ard.sendServo(9, 0, false);
		ard.sendDigital(18, ARD_LOW);  // pin 20 if using StandardFirmata from Arduino 0022 or older
		break;
	case ' ':
		//ard.resetEncoderPosition(0);
		ard.getAllEncoderPositions();
<<<<<<< HEAD
=======
		break;
	case '1':
		//get the stepper position from the target position in steps
		ard.getStepperDistanceFrom(0);
		break;
	case '2':
		//get the current position of the stepper
		ard.getStepperPosition(0);
		break;
	case '3':
		//set the speed in terms of steps per second 
		//in terms of RPM = steps per second / steps per revolution 
		ard.setStepperSpeed(0, 20000);
		break;
	case '4':
		//set the speed in terms of steps per second 
		//in terms of RPM = steps per second / steps per revolution
		ard.setStepperSpeed(0, 10000);
		break;
	case '5':
		//set the speed in terms of steps per second 
		//in terms of RPM = steps per second / steps per revolution
		ard.setStepperSpeed(0, 5000);
		break;
	case '6':
		//set the acceleration in terms of steps per second per second
		//in terms of every second the motor speeds up by steps per second / steps per revolution
		ard.setStepperAcceleration(0, 0);
		break;
	case '7':
		//set the acceleration in terms of steps per second per second
		//in terms of every second the motor speeds up by steps per second / steps per revolution
		ard.setStepperAcceleration(0, 1000);
		break;
	case '8':
		//set the deceleration in terms of steps per second per second
		//in terms of every second the motor slows up by steps per second / steps per revolution
		ard.setStepperDeceleration(0, 0);
		break;
	case '9':
		//set the deceleration in terms of steps per second per second
		//in terms of every second the motor slows up by steps per second / steps per revolution
		ard.setStepperDeceleration(0, 1000);
		break;
>>>>>>> origin/experimental
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	// turn on the onboard LED when the application window is clicked
	ard.sendDigital(13, ARD_HIGH);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	// turn off the onboard LED when the application window is clicked
	ard.sendDigital(13, ARD_LOW);
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}