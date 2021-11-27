/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/DriveController.h"

/**
 * Constructor
 * @param robot a RobotModel
 * @param humanControl a ControlBoard
 */
DriveController::DriveController(RobotModel *robot, ControlBoard *humanControl) :
    driveLayout_(robot->GetDriverTab().GetLayout("Drive Modes", "List Layout").WithPosition(0, 1))
    {

    // initialize class variables
    robot_ = robot;
    humanControl_ = humanControl;
    
    arcadeMode_ = true; 

    thrustSensitivity_ = 0.0;
    rotateSensitivity_ = 0.0;

    rightJoystickXLastValue_ = 0.0;
    rightJoystickXCurrValue_ = 0.0;

    minForwardThrust_ = minBackwardThrust_ = 0.0;

    // Setup shuffleboard entries
    arcadeEntry_ = driveLayout_.Add("Arcade Mode", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    thrustSensitivityEntry_ = driveLayout_.Add("Thrust Sensitivity", 0.5).GetEntry();
    rotateSensitivityEntry_ = driveLayout_.Add("Rotate Sensitivity", 0.7).GetEntry();
    anaModeEntry_ = driveLayout_.Add("Ana Mode", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    autoShiftEntry_ = robot_->GetFunctionalityTab().Add("auto shift", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    highGearEntry_ = robot_->GetFunctionalityTab().Add("high gear", robot_->IsHighGear()).GetEntry();
    
    printf("end of drive controller constructor\n");
}

/**
 * Periodic update, updates shuffleboard entries, drive (arcade versus tank), and gear shifts
 */
void DriveController::Update(){
    // update shuffleboard entries
    RefreshShuffleboard();

    // get x and y values from both joysticks
    double leftJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kX);
    double rightJoyX = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kX);
    double leftJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kLeftJoy, ControlBoard::Axes::kY);
    double rightJoyY = humanControl_->GetJoystickValue(ControlBoard::Joysticks::kRightJoy, ControlBoard::Axes::kY);

    // set to tank drive or arcade drive
    if (arcadeMode_){
        ArcadeDrive(leftJoyY, rightJoyX);
    } else {
        TankDrive(leftJoyY, rightJoyY);
    }

    // shift gear based on shuffleboard switch
    if (autoShiftEntry_.GetBoolean(true)){
        if (humanControl_->GetDesired(ControlBoard::Buttons::kGearShiftButton)) {
            robot_->GearShift();
        } else {
            robot_->SetLowGear();
        }
    } else {
        if (humanControl_->GetDesired(ControlBoard::Buttons::kGearShiftButton)) {
            robot_->SetHighGear();
        } else {
            robot_->SetLowGear();
        }
    }
        

}

/** 
 * Updates drive controller related shuffleboard entries
 */ 
void DriveController::RefreshShuffleboard(){
    thrustSensitivity_ = thrustSensitivityEntry_.GetDouble(0.0);
    rotateSensitivity_ = rotateSensitivityEntry_.GetDouble(0.0);
    arcadeMode_ = arcadeEntry_.GetBoolean(true);
    highGearEntry_.SetBoolean(robot_->IsHighGear());
}

void DriveController::Reset(){}

/** 
 * Tank drive
 * @param left a double for left output
 * @param right a double for right output
 */
void DriveController::TankDrive(double left, double right){
    // adjusting sensitivity
    left = GetDeadbandAdjustment(left);
    left = GetCubicAdjustment(left, thrustSensitivity_);
    right = GetDeadbandAdjustment(right);
    right = GetCubicAdjustment(right, rotateSensitivity_);

    MaxSpeedAdjustment(left, right);
    FrictionAdjustment(left,right, true);

    robot_->SetDriveValues(left, right);
}

/** 
 * Arcade drive
 * @param thrust a double
 * @param rotate a double
 */
void DriveController::ArcadeDrive(double thrust, double rotate){
    // adjusting sensitivity for turn
    thrust = GetDeadbandAdjustment(thrust);
    thrust = GetCubicAdjustment(thrust, thrustSensitivity_);
    rotate = GetDeadbandAdjustment(rotate);
    rotate = GetCubicAdjustment(rotate, rotateSensitivity_);
    
    double rotationValueAdjustment = GetRotateVelocityAdjustment(rotate);

    double leftOutput, rightOutput;

    // depending on drive mode, calculate outputs and set motors in drivetrain
    if(anaModeEntry_.GetBoolean(true) || (!anaModeEntry_.GetBoolean(true) && thrust >= 0.0)){
		leftOutput = thrust + rotate;		
		rightOutput = thrust - rotate*(1+rotationValueAdjustment);
	} else {
		leftOutput = thrust - rotate;
		rightOutput = thrust + rotate*(1+rotationValueAdjustment);
	}

    MaxSpeedAdjustment(leftOutput, rightOutput);
    FrictionAdjustment(leftOutput, rightOutput, true);
    
    robot_->SetDriveValues(leftOutput, rightOutput);
}

/** 
 * Adjusts joystick sensitivity using a cubic for smoother driving
 * @param value a double
 * @param adjustmentConstant a double
 * @return adjusted value as a double
 */
double DriveController::GetCubicAdjustment(double value, double adjustmentConstant){
    return adjustmentConstant * std::pow(value, 3.0) + (1.0 - adjustmentConstant) * value;
}

/** 
 * Adjusts rotation for turns in arcade drive
 * @param value a double
 * @return adjusted value as a double
 */
double DriveController::GetRotateVelocityAdjustment(double value){
    rightJoystickXLastValue_ = rightJoystickXCurrValue_;
    rightJoystickXCurrValue_ = value;
    double time = 60.0/50; // time between last iteration and current
    return abs(rightJoystickXCurrValue_-rightJoystickXLastValue_)/time;
}

/** 
 * Returns how much the thrust value should be adjusted: if it's lower than the deadband, the robot should not move
 * @param value a double
 * @return adjusted value as a double
 */
double DriveController::GetDeadbandAdjustment(double value){
    if(fabs(value)<DEADBAND_MAX){
        return 0.0;
    }
    else if (value > DEADBAND_MAX){ // robot power is 0.0-1.0
        return (1/(1-DEADBAND_MAX))*(value - DEADBAND_MAX); // fits any deadband value
    }
    else{
        return (1/(1-DEADBAND_MAX))*(value + DEADBAND_MAX);
    }
}

/** 
 * Adjusts left and right drive values since robot power must be in the range -1.0 to 1.0
 * @param leftValue a double
 * @param rightValue a double
 */
void DriveController::MaxSpeedAdjustment(double &leftvalue, double &rightvalue){
    if (leftvalue > 1.0){
        rightvalue /= leftvalue;
        leftvalue = 1.0;
    } else if (leftvalue < -1.0){
        rightvalue /= -leftvalue;
        leftvalue = -1.0;
    }
    if (rightvalue > 1.0){
        leftvalue /= rightvalue;
        rightvalue = 1.0;
    } else if (rightvalue < -1.0){
        leftvalue /= -rightvalue;
        rightvalue = -1.0;
    }
}

/** 
 * Gives extra power to motors to account for friction when robot begins to move
 * @param leftDrive a double
 * @param rightDrive a double
 * @param testMode a boolean
 */
void DriveController::FrictionAdjustment(double &leftDrive, double &rightDrive, bool testMode){
    
    if (leftDrive != 0.0 && robot_->GetLeftEncoderStopped()) {
        if (leftDrive > 0.0) {
            if (!testMode) {
                leftDrive = STATIC_FRICTION_DRIVE;
            }    
        } else {
            if (!testMode) {
                leftDrive = -STATIC_FRICTION_DRIVE;
            }
        }
    } else if (leftDrive != 0.0 && !robot_->GetLeftEncoderStopped()) {
        if (minForwardThrust_ == 0.0 && leftDrive < 0.0) {
            minForwardThrust_ = leftDrive;
        }
        if (minBackwardThrust_ == 0.0 && leftDrive > 0.0) {
            minBackwardThrust_ = leftDrive;
        }
    }
    
    if (rightDrive != 0.0 && robot_->GetRightEncoderStopped()) {
        if (rightDrive > 0.0) {
            if (!testMode) {
                rightDrive = STATIC_FRICTION_DRIVE;
            }
        } else {
            if (!testMode) {
                rightDrive = -STATIC_FRICTION_DRIVE;
            }
        }
    }

}

/**
 * Destructor
 */ 
DriveController::~DriveController(){
    arcadeEntry_.Delete();
    thrustSensitivityEntry_.Delete();
    rotateSensitivityEntry_.Delete();
    anaModeEntry_.Delete();
    autoShiftEntry_.Delete();
    highGearEntry_.Delete();
}