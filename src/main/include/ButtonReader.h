/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>

// this file outlines classes that read the states of buttons.
// ButtonReader reads the states of push buttons

class ButtonReader {
public:
	/**
	 * Constructor
	 * @param joystick
	 * @param buttonNum
	 */
	ButtonReader(frc::Joystick *joystick, int buttonNum);

	/**
	 * Destructor
	 */
	virtual ~ButtonReader();

	/** 
	 * Gets value of the button (pressed or not)
	 */
	void ReadValue();

	/** 
	 * Checks if the button is pressed (down is true)
	 * @return true if pressed
	 */
	bool IsDown();

	/**
	 * Checks if button was just pressed, only true at the moment when button is initially pressed
	 * @return true or false if button was just pressed
	 */
	bool WasJustPressed();

	/** 
	 * Checks if button has been released, only true at the moment when button is released
	 * @return true or false if button was just released
	 */
	bool WasJustReleased();

	/** 
	 * Checks if button has changed from being pressed to released or vice versa 
	 * @return true or false if state just changed
	 */
	bool StateJustChanged();

private:
	frc::Joystick *joystick;
	int buttonNum;

	// currState is true if button is currently pressed
	bool lastState;
	bool currState;
};

// ToggleButtonReader reads the states of toggles
class ToggleButtonReader : public ButtonReader {
public:
	/** 
	 * Constructor 
	 * - ToggleButtonReader is subclass of ButtonReader, its state toggles every time that it was just released, 
	 * and reads the state of toggles.
	 * @param joy
	 * @param buttonNum
	 */
	ToggleButtonReader(frc::Joystick *joy, int buttonNum);

	/**
	 * Destructor
	 */
	virtual ~ToggleButtonReader();

	/** 
	 * Toggles the button's state and then reads the button's state 
	 * @return true if currently on
	 */
	virtual bool GetState();

private:
	bool currToggleState;
};

enum SwitchState {
	kUp = 1,
	kNeutral = 0,
	kDown = -1,
};

// SwitchReaader reads the state of switches
class SwitchReader {
public:
	/**
	 * Constructor for SwitchReader, reads the state of switches (up, down, or neutral)
	 * @param joy
	 * @param upButton
	 * @param downButton
	 */
	SwitchReader(frc::Joystick *jooy, int upButton, int downButton);

	/**
	 * Destructor
	 */
	virtual ~SwitchReader();

	/** 
	 * Gets the switches state and returns its state (up, down, or neutral)
	 * @return current SwitchState (kUp or kDown or KNeutral)
	 */
	SwitchState GetSwitchState();

private:
	frc::Joystick *joy;
	int upB;
	int downB;
};
