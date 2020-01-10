/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/DriveController.h"

DriveController::DriveController(RobotModel *robot, ControlBoard *humanControl) :
    driveLayout_(robot->GetDriverTab().GetLayout("Drive Modes", "List Layout").WithPosition(0, 1))
    {
    robot_ = robot;
    humanControl_ = humanControl;
    
    arcadeMode_ = true;

    thrustSensitivity_ = 0.0;
    rotateSensitivity_ = 0.0;

    arcadeEntry_ = driveLayout_.Add("Arcade Mode", true).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
    thrustSensitivityEntry_ = driveLayout_.Add("Thrust Sensitivity", 0.0).GetEntry();
    rotateSensitivityEntry_ = driveLayout_.Add("Rotate Sensitivity", 0.0).GetEntry();
}

void DriveController::Update(){
    RefreshShuffleboard();

    double leftJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kX);
    double rightJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kX);
    double leftJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kY);
    double rightJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kY);
    
    if(arcadeMode_){
        ArcadeDrive(leftJoyY, rightJoyX, thrustSensitivity_, rotateSensitivity_); //TODO add shuffleboard option
    } else {
        TankDrive(leftJoyY, rightJoyY);
    }
}

void DriveController::RefreshShuffleboard(){
    thrustSensitivity_ = thrustSensitivityEntry_.GetDouble(0.0);
    rotateSensitivity_ = rotateSensitivityEntry_.GetDouble(0.0);
    arcadeMode_ = arcadeEntry_.GetBoolean(true);
}

void DriveController::TankDrive(double left, double right){
    left = GetDeadbandAdjustment(left);
    left = GetCubicAdjustment(left, thrustSensitivity_);
    right = GetDeadbandAdjustment(right);
    right = GetCubicAdjustment(right, rotateSensitivity_);
    
    MaxSpeedAdjustment(left, right);

    robot_->SetDriveValues(left, right);
}

void DriveController::ArcadeDrive(double thrust, double rotate, double thrustSensitivity, double rotateSensitivity){
    thrust = GetDeadbandAdjustment(thrust);
    thrust = GetCubicAdjustment(thrust, thrustSensitivity_);
    rotate = GetDeadbandAdjustment(rotate);
    rotate = GetCubicAdjustment(rotate, rotateSensitivity_);

    double leftOutput, rightOutput;
    if(thrust >= 0.0){
        leftOutput = thrust + rotate;		
		rightOutput = thrust- rotate;
	} else {
		leftOutput = thrust - rotate;
		rightOutput = thrust + rotate;
    }

    MaxSpeedAdjustment(leftOutput, rightOutput);

    robot_->SetDriveValues(leftOutput, rightOutput);
}

double DriveController::GetCubicAdjustment(double value, double adjustmentConstant){
    return adjustmentConstant * std::pow(value, 3.0) + (1.0 - adjustmentConstant) * value;
}

double DriveController::GetDeadbandAdjustment(double value){
    if(fabs(value) < DEADBAND_MAX){
        return 0.0;
    }
    return value;
}

void DriveController::MaxSpeedAdjustment(double &value1, double &value2){
    if(value1>1.0){
        value2 /= value1;
        value1 = 1.0;
    } else if(value1<-1.0){
        value2 /= -value1;
        value1 = -1.0;
    }
    if(value2>1.0){
        value1 /= value2;
        value2 = 1.0;
    } else if(value2<-1.0){
        value1 /= -value2;
        value2 = -1.0;
    }
}

DriveController::~DriveController(){}