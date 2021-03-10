/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ButtonReader.h"
#include <frc/WPILib.h>

// class ButtonReader
// constructs a joystick and sets the port of the button on the joystick, and reads the state of the button

ButtonReader::ButtonReader(frc::Joystick* myJoystick, int myButtonNum){
	joystick = myJoystick;
	buttonNum = myButtonNum;

	// initially set currState as value of the button, initially set lastState as currState
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

// gets value of the button (pressed or not)
void ButtonReader::ReadValue() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
}

// sees if button has been pressed, only true at the moment when button is initially pressed
bool ButtonReader::WasJustPressed() {
	return (lastState == false && currState == true);
}

// sees if button has been released, only true at the moment when button is released
bool ButtonReader::WasJustReleased() {
	return (lastState == true && currState == false);
}

// sees if button has changed from being pressed to released or vice versa 
bool ButtonReader::StateJustChanged() {
	return (lastState != currState);
}

// sees if the button is pressed (down is true)
bool ButtonReader::IsDown() {
	return currState;
}

ButtonReader::~ButtonReader() {
}

// class ToggleButtonReader
// is subclass of ButtonReader, its state toggles every time that it was just released, and reads the state of toggles.

ToggleButtonReader::ToggleButtonReader(frc::Joystick *joy, int buttonNum) :
	ButtonReader(joy, buttonNum) {
	currToggleState = false;
}

// toggles the button's state and then reads the button's state 
bool ToggleButtonReader::GetState() {
	// currToggleState is toggled every time the button was just released
	if (WasJustReleased()) {
		currToggleState = !currToggleState;
	}

	return (currToggleState);
}

ToggleButtonReader::~ToggleButtonReader() {
}

// class SwitchReader
// reads the state of switches (up, down, or neutral)

SwitchReader::SwitchReader(frc::Joystick *myJoy, int upButton, int downButton) {
	joy = myJoy;
	upB = upButton;
	downB = downButton;
}

// gets the switches state and returns its state (up, down, or neutral)
SwitchState SwitchReader::GetSwitchState() {
	if (joy->GetRawButton(upB))
		return kUp;
	if (joy->GetRawButton(downB))
		return kDown;
	return kNeutral;
}

SwitchReader::~SwitchReader() {
}