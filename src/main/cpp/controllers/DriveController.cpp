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

    rightJoystickXLastValue_ = 0.0;
    rightJoystickXCurrValue_ = 0.0;

    minForwardThrust_ = minBackwardThrust_ = 0.0;

    arcadeEntry_ = driveLayout_.Add("Arcade Mode", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    thrustSensitivityEntry_ = driveLayout_.Add("Thrust Sensitivity", 0.5).GetEntry();
    rotateSensitivityEntry_ = driveLayout_.Add("Rotate Sensitivity", 0.5).GetEntry();
    anaModeEntry_ = driveLayout_.Add("Ana Mode", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    autoShiftEntry_ = robot_->GetFunctionalityTab().Add("auto shift", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    highGearEntry_ = robot_->GetFunctionalityTab().Add("high gear", robot_->IsHighGear()).GetEntry();
    printf("end of drive controller constructor\n");
}

void DriveController::Update(){
    RefreshShuffleboard();

    double leftJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kX);
    double rightJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kX);
    double leftJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kY);
    double rightJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kY);
    
    //std::cout << "drive: " << leftJoyY << " turn: " << rightJoyX << std::endl;


    if(arcadeMode_){
        ArcadeDrive(leftJoyY, rightJoyX, thrustSensitivity_, rotateSensitivity_); //test turn, might need to negatize rightJoyX
    } else {
        TankDrive(leftJoyY, rightJoyY);
    }

    if(autoShiftEntry_.GetBoolean(true)){
        if(humanControl_->GetDesired(ControlBoard::Buttons::kGearShiftButton)) {
            robot_->GearShift();
        } else {
            robot_->SetLowGear();
        }
    } else {
        if(humanControl_->GetDesired(ControlBoard::Buttons::kGearShiftButton)) {
            robot_->SetHighGear();
        } else {
            robot_->SetLowGear();
        }
    }
        

}

void DriveController::RefreshShuffleboard(){
    thrustSensitivity_ = thrustSensitivityEntry_.GetDouble(0.0);
    rotateSensitivity_ = rotateSensitivityEntry_.GetDouble(0.0);
    arcadeMode_ = arcadeEntry_.GetBoolean(true);
    highGearEntry_.SetBoolean(robot_->IsHighGear());
}

void DriveController::Reset(){
    
}

void DriveController::TankDrive(double left, double right){
    left = GetDeadbandAdjustment(left);
    left = GetCubicAdjustment(left, thrustSensitivity_);
    right = GetDeadbandAdjustment(right);
    std::cout<< "right dbaj: " << right << " left dbaj: " << left << std::endl;
    right = GetCubicAdjustment(right, rotateSensitivity_);
    MaxSpeedAdjustment(left, right);
    FrictionAdjustment(left,right, true);

    robot_->SetDriveValues(left, right);
}

void DriveController::ArcadeDrive(double thrust, double rotate, double thrustSensitivity, double rotateSensitivity){
    //std::cout<< "thrust before:"<<  thrust  << std::endl;
    thrust = GetDeadbandAdjustment(thrust);
    //std::cout<< "thrust deadband adj: " <<  thrust  << std::endl;
    thrust = GetCubicAdjustment(thrust, thrustSensitivity_); //todo logic error, pass in param but use class var
    rotate = GetDeadbandAdjustment(rotate);
    rotate = GetCubicAdjustment(rotate, rotateSensitivity_);
    
    double rotationValueAdjustment = GetRotateVelocityAdjustment(rotate);

    double leftOutput, rightOutput;
    // if(thrust >= 0.0){
    //     leftOutput = thrust + rotate;		
	// 	rightOutput = thrust - rotate*(1+rotationValueAdjustment);
	// } else {
	// 	leftOutput = thrust - rotate;
	// 	rightOutput = thrust + rotate*(1+rotationValueAdjustment);
    // }

    if(anaModeEntry_.GetBoolean(true) || (!anaModeEntry_.GetBoolean(true) && thrust >= 0.0)){
	// || reverseReverseNet_.GetBoolean(false) && thrustValue > 0.0){ (for lili mode)
		leftOutput = thrust + rotate;		
		rightOutput = thrust - rotate*(1+rotationValueAdjustment);
	} else {
		leftOutput = thrust - rotate;
		rightOutput = thrust + rotate*(1+rotationValueAdjustment);
	}

    MaxSpeedAdjustment(leftOutput, rightOutput);
    FrictionAdjustment(leftOutput, rightOutput, true);
    //robot_->GearShift();
    
    robot_->SetDriveValues(leftOutput, rightOutput);
    //printf("Left Output: %f, Right Output: %f", leftOutput, rightOutput);
}

double DriveController::GetCubicAdjustment(double value, double adjustmentConstant){
    return adjustmentConstant * std::pow(value, 3.0) + (1.0 - adjustmentConstant) * value;
}

double DriveController::GetRotateVelocityAdjustment(double value){
    rightJoystickXLastValue_ = rightJoystickXCurrValue_;
    rightJoystickXCurrValue_ = value;
    double time = 60.0/50;
    return abs(rightJoystickXCurrValue_-rightJoystickXLastValue_)/time;
}


double DriveController::GetDeadbandAdjustment(double value){
    if(fabs(value)<DEADBAND_MAX){
        return 0.0;
    }
    else if (value > DEADBAND_MAX){ //REMAPPED JOYSTICK RANGE FOR ROBOT POWER FROM 0.0-1.0
        return (1/(1-DEADBAND_MAX))*(value - DEADBAND_MAX); //adjusted to fit any deadband value
        //return (10.0/9)*value - (1.0/9);
    }
    else{
        return (1/(1-DEADBAND_MAX))*(value + DEADBAND_MAX);
        //return (10.0/9)*value + (1.0/9);
    }

    // if(fabs(value) < DEADBAND_MAX){
    //     return 0.0;
    // }
    // return value;
}

void DriveController::MaxSpeedAdjustment(double &leftvalue, double &rightvalue){
    if(leftvalue>1.0){
        rightvalue /= leftvalue;
        leftvalue = 1.0;
    } else if(leftvalue<-1.0){
        rightvalue /= -leftvalue;
        leftvalue = -1.0;
    }
    if(rightvalue>1.0){
        leftvalue /= rightvalue;
        rightvalue = 1.0;
    } else if(rightvalue<-1.0){
        leftvalue /= -rightvalue;
        rightvalue = -1.0;
    }
}

void DriveController::FrictionAdjustment(double &leftDrive, double &rightDrive, bool testMode){
    //std::cout<< "leftOutput: " << leftDrive << "rightOutput: " << rightDrive << std::endl;
    //std::cout<< "left stop: " << robot_->GetLeftEncoderStopped() << " right stop: " << robot_->GetRightEncoderStopped() << std::endl;
    //std::cout<< "left velocity: " << robot_->GetLeftVelocity() << std::endl;
    if (leftDrive != 0.0 && robot_->GetLeftEncoderStopped()) {
        if (leftDrive > 0.0) {
            //std::cout << "left drive pos, encoder stopped" << std::endl;
            if (!testMode) {
                leftDrive = STATIC_FRICTION_DRIVE;
            }    
        }
        else {
            //std::cout << "left drive neg, encoder stopped" << std::endl;
            if (!testMode){
                leftDrive = -STATIC_FRICTION_DRIVE;
            }
        }
    }
    else if (leftDrive != 0.0 && !robot_->GetLeftEncoderStopped()) {
        //std::cout<< "hi" << std::endl;
        if (minForwardThrust_ == 0.0 && leftDrive < 0.0) {
            minForwardThrust_ = leftDrive;
        }
        if (minBackwardThrust_ == 0.0 && leftDrive > 0.0) {
            minBackwardThrust_ = leftDrive;
        }
        if (testMode){
        //std::cout << "minForward: " << minForwardThrust_ << " minBackward: " << minBackwardThrust_ << std::endl;
        }
    }
    // else{
    //     std::cout << "no left friction adjust needed" << std::endl;
    //     std::cout << "left output:" << leftDrive << " left velocity:" << robot_->GetLeftVelocity() <<std::endl;
    // }
    if (rightDrive != 0.0 && robot_->GetRightEncoderStopped()) {
        if (rightDrive > 0.0) {
            if (!testMode){
                rightDrive = STATIC_FRICTION_DRIVE;
            }
        }
        else {
            if (!testMode){
                rightDrive = -STATIC_FRICTION_DRIVE;
            }
        }
    }
    // else{
    //     std::cout << "no right friction adjust needed" << std::endl;
    // }
}

DriveController::~DriveController(){}