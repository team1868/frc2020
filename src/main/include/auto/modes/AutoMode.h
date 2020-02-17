/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "ControlBoard.h"
#include "controllers/SuperstructureController.h" // this could be why the code is crashing lmao
#include "../commands/CurveCommand.h"
#include "auto/commands/DriveStraightCommand.h"
#include "auto/commands/PivotCommand.h"
//#include "../commands/WaitingCommand.h"
#include "auto/PIDSource/PIDInputSource.h"
#include "auto/commands/PointCommand.h"
#include "auto/PIDSource/PIDOutputSource.h"

class AutoMode {
  public:
    enum AutoPositions {kBlank, kLeft, kMiddle, kRight};

    AutoMode(RobotModel *robot, ControlBoard *controlBoard);
    // might need to comment out the robot and controlBoard up here because superstructure controller takes in robot and controlboard as parameters when created
    virtual ~AutoMode();

    virtual void CreateQueue(AutoMode::AutoPositions pos) {};

    void QueueFromString(string autoSequence);

    AutoCommand* GetStringCommand(char command);

    bool IsFailed(char command);

    virtual void Init() = 0;

    void Update(double currTimeSec, double deltaTimeSec);

    bool IsDone();

    bool Abort();

	  void Disable();

  protected:
    AutoCommand *firstCommand_;
	  AutoCommand *currentCommand_;
	  RobotModel* robot_;
    ControlBoard *humanControl_;

	  NavXPIDSource* navX_;
	  TalonEncoderPIDSource* talonEncoder_;
    TalonEncoderCurvePIDSource* talonEncoderCurve_;

	  AnglePIDOutput *angleOutput_;
	  DistancePIDOutput *distanceOutput_;

	  istringstream iss;
	  bool breakDesired_;
	  double currAngle_;
};