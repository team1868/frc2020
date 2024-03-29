/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ControlBoard.h"

/**
 * Constructor - 
 * sets each button to specific port & usb port
 */
ControlBoard::ControlBoard() {
	// reset joystick coordinates
    leftJoyX_ = 0.0;
    leftJoyY_ = 0.0;
    rightJoyX_ = 0.0;
    rightJoyY_ = 0.0;

	// assign joysticks to ports
    leftJoy_ = new frc::Joystick(LEFT_JOY_USB_PORT);
    rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT); 
    operatorJoy_ = new frc::Joystick(OPERATOR_JOY_USB_PORT);
    operatorJoyB_ = new frc::Joystick(OPERATOR_JOY_B_USB_PORT);

	// create list of buttons
	buttons_ = std::unordered_map<Buttons, ButtonReader*>();
	
	// BUTTONS
	// chassis movement buttons
	buttons_[kAlignButton] = new ButtonReader(rightJoy_, ALIGN_TAPE_BUTTON_PORT);
	buttons_[kGearShiftButton] = new ButtonReader(leftJoy_, GEARSHIFT_BUTTON_PORT);
	// ball mechanism controls
	buttons_[kIntakeSeriesButton] = new ButtonReader(operatorJoyB_, INTAKE_SERIES_BUTTON_PORT);
	buttons_[kIndexSeriesButton] = new ButtonReader(operatorJoyB_, INDEX_SERIES_BUTTON_PORT);
	// shoot controls
	buttons_[kShootClosePrepButton] = new ButtonReader(operatorJoyB_, SHOOT_CLOSE_PREP_BUTTON_PORT);
	buttons_[kShootFarPrepButton] = new ButtonReader(operatorJoyB_, SHOOT_FAR_PREP_BUTTON_PORT);
	buttons_[kShootingButton] = new ButtonReader(operatorJoy_, SHOOTING_BUTTON_PORT);
	// climb controls
	buttons_[kClimbRightElevatorUpButton] = new ButtonReader(operatorJoyB_, CLIMB_RIGHT_ELEVATOR_UP_BUTTON_PORT);
	buttons_[kClimbRightElevatorDownButton] = new ButtonReader(operatorJoyB_, CLIMB_RIGHT_ELEVATOR_DOWN_BUTTON_PORT);
	buttons_[kClimbLeftElevatorUpButton] = new ButtonReader(operatorJoy_, CLIMB_LEFT_ELEVATOR_UP_BUTTON_PORT);
	buttons_[kClimbLeftElevatorDownButton] = new ButtonReader(operatorJoy_, CLIMB_LEFT_ELEVATOR_DOWN_BUTTON_PORT);
	buttons_[kUndoElevatorButton] = new ButtonReader(operatorJoyB_, UNDO_ELEVATOR_BUTTON_PORT);
	// wrist controls
	buttons_[kWristUpButton] = new ButtonReader(operatorJoy_, WRIST_UP_BUTTON_PORT);
	buttons_[kWristDownButton] = new ButtonReader(operatorJoy_, WRIST_DOWN_BUTTON_PORT);
    buttons_[kRunRollersButton] = new ButtonReader(operatorJoy_, WRIST_RUN_ROLLERS_BUTTON);
    buttons_[kReverseRollersButton] = new ButtonReader(operatorJoy_, WRIST_REVERSE_ROLLERS_BUTTON);
	// control panel controls
	buttons_[kControlPanelButton] = new ButtonReader(operatorJoy_, CONTROL_PANEL_BUTTON_PORT);

	// testing
	buttons_[kTestFarShootButton] = new ButtonReader(operatorJoy_, 1);
	
	// create buttons here after creating a option in the Buttons enum
	// example:
	// buttons_[kYourButton] = new ButtonReader(joystickname, BUTTON_PORT);
	
    ReadControls();
	printf("end of control board constructor\ns");
}

/**
 * Gets joystick values given joystick and a specific axis
 * @param j left or right joystick (kLeftJoy, kRightJoy)
 * @param a x or y axis (kX, kY are the acceptable axes)
 * @return desired joystick's position on specified axis(x or y)
 */
double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
	  case (kLeftJoy):
		switch(a) {
			case(kX):
				return leftJoyX_;
			case(kY):
				return leftJoyY_;
            default:
                printf("WARNING: left joystick value not received");
		}
	  	break;
	  case (kRightJoy):
		switch(a){
	  	    case(kX):
			    return rightJoyX_;
	  	    case(kY):
			    return rightJoyY_;
            default:
                printf("WARNING: right joystick value not received");
		}
		break;
	  default:
      printf("WARNING: Joystick value not received in ControlBoard::GetJoystickValue\n");
	}
	return 0;
}

/** 
 * Updates joystick variables to current joystick x & y coordinates
 */
void ControlBoard::ReadControls(){
    ReadAllButtons();

    leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = -leftJoy_->GetY();
    rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = -rightJoy_->GetY();
}

/** 
 * Checks whether specific button is down
 * @param button a specific button
 * @return whether button is currently down aka value of button
 */
bool ControlBoard::GetDesired(Buttons button){
	return buttons_[button]->IsDown();
}

/**
  * Checks if specific button was just pressed
  * @param button a specific button
  * @return only true when button is initially pressed
  */
bool ControlBoard::JustPressed(Buttons button){
	return buttons_[button]->WasJustPressed();
}

/**
 * Reads & updates states(lastState & currState) of all buttons
 */
void ControlBoard::ReadAllButtons(){
	for(std::pair<Buttons, ButtonReader*> b : buttons_){
		b.second->ReadValue();
	}
}

/**
 * Destructor
 */
ControlBoard::~ControlBoard(){}
