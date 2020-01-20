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

class DriveController {
  public:
    DriveController(RobotModel *robot, ControlBoard *humanControl);

    void Update();
    void RefreshShuffleboard();
    void TankDrive(double left, double right);
    void ArcadeDrive(double thrust, double rotate, double thrustSensitivity, double rotateSensitivity);

    // adjusts sensitivity for turn
  	double GetCubicAdjustment(double value, double adjustmentConstant);
    double GetDeadbandAdjustment(double value);
    double GetRotateVelocityAdjustment(double value);
    void MaxSpeedAdjustment(double &leftvalue, double &rightvalue);

    ~DriveController();
    
  private:
    RobotModel *robot_;
	  ControlBoard *humanControl_;
    bool arcadeMode_;

    ShuffleboardLayout &driveLayout_;
    nt::NetworkTableEntry arcadeEntry_, thrustSensitivityEntry_, rotateSensitivityEntry_;
    
    double thrustSensitivity_, rotateSensitivity_;
    double rightJoystickXLastValue_, rightJoystickXCurrValue_;
};
