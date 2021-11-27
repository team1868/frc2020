/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotModel.h"
#include "ControlBoard.h"

static const double DEADBAND_MAX = 0.1;
static const double STATIC_FRICTION_DRIVE = 0.07; //NEED TO CALIBRATE 

class DriveController {
  public:
    /**
     * Constructor
     * @param robot a RobotModel
     * @param humanControl a ControlBoard
     */
    DriveController(RobotModel *robot, ControlBoard *humanControl);

    /**
     * Periodic update, updates shuffleboard entries, drive (arcade versus tank), and gear shifts
     */
    void Update();

    void Reset();

    /** 
     * Updates drive controller related shuffleboard entries
     */ 
    void RefreshShuffleboard();

    /** 
     * Tank drive
     * @param left a double for left output
     * @param right a double for right output
     */
    void TankDrive(double left, double right);

    /** 
     * Arcade drive
     * @param thrust a double
     * @param rotate a double
     */
    void ArcadeDrive(double thrust, double rotate);

    /** 
     * Adjusts joystick sensitivity using a cubic for smoother driving
     * @param value a double
     * @param adjustmentConstant a double
     * @return adjusted value as a double
     */
  	double GetCubicAdjustment(double value, double adjustmentConstant);

    /** 
     * Returns how much the thrust value should be adjusted: if it's lower than the deadband, the robot should not move
     * @param value a double
     * @return adjusted value as a double
     */
    double GetDeadbandAdjustment(double value);

    /** 
     * Gives extra power to motors to account for friction when robot begins to move
     * @param leftDrive a double
     * @param rightDrive a double
     * @param testMode a boolean
     */
    void FrictionAdjustment(double &leftDrive, double &rightDrive, bool testMode);
    
    /** 
     * Adjusts rotation for turns in arcade drive
     * @param value a double
     * @return adjusted value as a double
     */
    double GetRotateVelocityAdjustment(double value);

    /** 
     * Adjusts left and right drive values since robot power must be in the range -1.0 to 1.0
     * @param leftValue a double
     * @param rightValue a double
     */
    void MaxSpeedAdjustment(double &leftvalue, double &rightvalue);

    /**
     * Destructor
     */ 
    ~DriveController();
    
  private:
    RobotModel *robot_;
	  ControlBoard *humanControl_;
    bool arcadeMode_;

    // shuffleboard
    frc::ShuffleboardLayout &driveLayout_;
    nt::NetworkTableEntry arcadeEntry_, thrustSensitivityEntry_, rotateSensitivityEntry_, anaModeEntry_, autoShiftEntry_, highGearEntry_;
    
    // joystick and drive values
    double thrustSensitivity_, rotateSensitivity_;
    double rightJoystickXLastValue_, rightJoystickXCurrValue_;
    double minForwardThrust_, minBackwardThrust_;
};
