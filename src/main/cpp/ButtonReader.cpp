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

ButtonReader::ButtonReader(frc::Joystick* myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

void ButtonReader::ReadValue() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
}

bool ButtonReader::WasJustPressed() {
	return (lastState == false && currState == true);
}

bool ButtonReader::WasJustReleased() {
	return (lastState == true && currState == false);
}

bool ButtonReader::StateJustChanged() {
	return (lastState != currState);
}

bool ButtonReader::IsDown() {
	return currState;
}

ButtonReader::~ButtonReader() {
}

ToggleButtonReader::ToggleButtonReader(frc::Joystick *joy, int buttonNum) :
	ButtonReader(joy, buttonNum) {
	currToggleState = false;
}

// gets the current state of the toggle
bool ToggleButtonReader::GetState() {
	if (WasJustReleased()) {
		currToggleState = !currToggleState;
	}
	return (currToggleState);
}

ToggleButtonReader::~ToggleButtonReader() {
}

SwitchReader::SwitchReader(frc::Joystick *myJoy, int upButton, int downButton) {
	joy = myJoy;
	upB = upButton;
	downB = downButton;
}

SwitchState SwitchReader::GetSwitchState() {
	if (joy->GetRawButton(upB))
		return kUp;
	if (joy->GetRawButton(downB))
		return kDown;
	return kNeutral;
}