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

	buttons_[kAlignButton] = new ButtonReader(rightJoy_, ALIGN_TAPE_BUTTON_PORT);
	buttons_[kTrenchAlignButton] = new ButtonReader(rightJoy_, TRENCH_ALIGN_TAPE_BUTTON_PORT);
	buttons_[kShootingButton] = new ButtonReader(operatorJoy_, SHOOTING_BUTTON_PORT);
	buttons_[kClimbElevatorUpButton] = new ButtonReader(operatorJoy_, CLIMB_ELEVATOR_UP_BUTTON_PORT);
	buttons_[kClimbElevatorDownButton] = new ButtonReader(operatorJoy_, CLIMB_ELEVATOR_DOWN_BUTTON_PORT);
	buttons_[kControlPanelStage2Button] = new ButtonReader(operatorJoy_, CONTROL_PANEL_STAGE_2_BUTTON_PORT);
	buttons_[kControlPanelStage3Button] = new ButtonReader(operatorJoyB_, CONTROL_PANEL_STAGE_3_BUTTON_PORT);
	buttons_[kIntakeSeriesButton] = new ButtonReader(operatorJoyB_, INTAKE_SERIES_BUTTON_PORT);
	buttons_[kIndexSeriesButton] = new ButtonReader(operatorJoyB_, INDEX_SERIES_BUTTON_PORT);
	buttons_[kShootClosePrepButton] = new ButtonReader(operatorJoyB_, SHOOT_CLOSE_PREP_BUTTON_PORT);
	buttons_[kShootFarPrepButton] = new ButtonReader(operatorJoyB_, SHOOT_FAR_PREP_BUTTON_PORT);

	buttons_[kClimbWinchLeftButton] = new ButtonReader(operatorJoy_, CLIMB_WINCH_RIGHT_BUTTON_PORT);
	buttons_[kClimbWinchRightButton] = new ButtonReader(operatorJoy_, CLIMB_WINCH_LEFT_BUTTON_PORT);

	//create buttons here after creating a option in the Buttons enum
	//example
	//buttons_[kYourButton] = new ButtonReader(joystickname, BUTTON_PORT);
	
    ReadControls();
	printf("end of control board constructor\ns");
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
}

void ControlBoard::ReadAllButtons(){
	for(std::pair<Buttons, ButtonReader*> b : buttons_){
		b.second->ReadValue();
	}
}

bool ControlBoard::GetDesired(Buttons button){
	printf("%d", buttons_[button]->IsDown(), "/n");
	return buttons_[button]->IsDown();
}

bool ControlBoard::JustPressed(Buttons button){
	return buttons_[button]->WasJustPressed();
}