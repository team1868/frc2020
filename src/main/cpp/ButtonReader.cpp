/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ButtonReader.h" 
#include <frc/WPILib.h>

/**
 * Constructor
 * @param joystick
 * @param buttonNum
 */
ButtonReader::ButtonReader(frc::Joystick* joystick, int buttonNum){
	joystick = joystick;
	buttonNum = buttonNum;

	// initially set currState as value of the button, initially set lastState as currState
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

/** 
 * Gets value of the button (pressed or not)
 */
void ButtonReader::ReadValue() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
}

/**
 * Checks if button was just pressed, only true at the moment when button is initially pressed
 * @return true or false if button was just pressed
 */
bool ButtonReader::WasJustPressed() {
	return (lastState == false && currState == true);
}

/** 
 * Checks if button has been released, only true at the moment when button is released
 * @return true or false if button was just released
 */
bool ButtonReader::WasJustReleased() {
	return (lastState == true && currState == false);
}

/** 
 * Checks if button has changed from being pressed to released or vice versa 
 * @return true or false if state just changed
 */
bool ButtonReader::StateJustChanged() {
	return (lastState != currState);
}

/** 
 * Checks if the button is pressed (down is true)
 * @return true if pressed
 */
bool ButtonReader::IsDown() {
	return currState;
}

/**
 * Destructor
 */ 
ButtonReader::~ButtonReader() {}


/** 
 * Constructor 
 * - ToggleButtonReader is subclass of ButtonReader, its state toggles every time that it was just released, 
 * and reads the state of toggles.
 * @param joy
 * @param buttonNum
 */
ToggleButtonReader::ToggleButtonReader(frc::Joystick *joy, int buttonNum) :
	ButtonReader(joy, buttonNum) {
	currToggleState = false;
}

/** 
 * Toggles the button's state and then reads the button's state 
 * @return true if currently on
 */
bool ToggleButtonReader::GetState() {
	// currToggleState is toggled every time the button was just released
	if (WasJustReleased()) {
		currToggleState = !currToggleState;
	}

	return (currToggleState);
}

/**
 * Destructor
 */
ToggleButtonReader::~ToggleButtonReader() {}

/**
 * Constructor for SwitchReader, reads the state of switches (up, down, or neutral)
 * @param joy
 * @param upButton
 * @param downButton
 */
SwitchReader::SwitchReader(frc::Joystick *joy, int upButton, int downButton) {
	joy = joy;
	upB = upButton;
	downB = downButton;
}

/** 
 * Gets the switches state and returns its state (up, down, or neutral)
 * @return current SwitchState (kUp or kDown or KNeutral)
 */
SwitchState SwitchReader::GetSwitchState() {
	if (joy->GetRawButton(upB))
		return kUp;
	if (joy->GetRawButton(downB))
		return kDown;
	return kNeutral;
}

/**
 * Destructor
 */
SwitchReader::~SwitchReader() {}