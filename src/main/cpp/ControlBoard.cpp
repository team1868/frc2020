/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ControlBoard.h"

ControlBoard::ControlBoard() {
    leftJoyX_ = 0.0;
    leftJoyY_ = 0.0;
    rightJoyX_ = 0.0;
    rightJoyY_ = 0.0;

    leftJoy_ = new frc::Joystick(LEFT_JOY_USB_PORT);
    rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT); 

    operatorJoy_ = new frc::Joystick(OPERATOR_JOY_USB_PORT);
    operatorJoyB_ = new frc::Joystick(OPERATOR_JOY_B_USB_PORT);

	buttons_ = std::unordered_map<Buttons, ButtonReader*>();
	
	//create buttons here after creating a option in the Buttons enum
	//example
	//buttons_[kYourButton] = new ButtonReader(joystickname, BUTTON_PORT);

	buttons_[kHighGearShift] = new ButtonReader(leftJoy_, HIGH_GEAR_BUTTON_PORT);
	buttons_[kLowGearShift] = new ButtonReader(leftJoy_, LOW_GEAR_BUTTON_PORT);
	buttons_[kAlignButton] = new ButtonReader(rightJoy_, ALIGN_TAPE_BUTTON_PORT);
	buttons_[kTrenchAlignButton] = new ButtonReader(rightJoy_, TRENCH_ALIGN_TAPE_BUTTON_PORT);

	flywheelDesired_ = false;
	climberDesired_ = false;
	flywheelButton_ = new ButtonReader(operatorJoy_, FLYWHEEL_BUTTON_PORT);
	climberButton_ = new ButtonReader(operatorJoy_, CLIMBER_BUTTON_PORT);
	//alignButton_ = new ButtonReader(rightJoy_, ALIGN_TAPE_BUTTON_PORT);
	
    ReadControls();
}

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

void ControlBoard::ReadControls(){
    ReadAllButtons();

    leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = -leftJoy_->GetY();
    rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = -rightJoy_->GetY();
	flywheelDesired_ = flywheelButton_ -> IsDown();
	climberDesired_ = climberButton_ -> IsDown();
}

bool ControlBoard::GetFlywheelDesired() {
	return flywheelDesired_;
}

bool ControlBoard::GetClimberDesired() {
	return climberDesired_;
}


void ControlBoard::ReadAllButtons(){
	for(std::pair<Buttons, ButtonReader*> b : buttons_){
		b.second->ReadValue();
	}
}

bool ControlBoard::GetDesired(Buttons button){
	return buttons_[button]->IsDown();
}

bool ControlBoard::JustPressed(Buttons button){
	return buttons_[button]->WasJustPressed();
}