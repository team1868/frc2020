/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "auto/AutoCommand.h"
#include "auto/commands/PivotCommand.h"
#include "auto/PIDsource/PIDInputSource.h"

/**
 * This class is called to align the robot to a retroreflective tape target, 
 * by reading the delta angle to the tape and pivoting to it. Driving towards
 * the target is not included or necessary.
*/
class AlignTapeCommand : public AutoCommand{
 public:

  // constructor
  AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource_, PivotPIDTalonOutput *talonOutput);
  
  // constructor
  AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource_);
  
  // destructor
  ~AlignTapeCommand();

  // initializes class variables
  void Init();

  // periodic update while executing command
  void Update(double currTimeSec, double deltaTimeSec);
  
  // check if the robot is aligned
  bool IsDone();

  // called when the command is done, destroys pivot class to prevent memory leaks
  void Reset();

 private:

  //classes used pointers
  RobotModel *robot_;
  PivotCommand *pivotCommand_;
  NavXPIDSource *navXSource_;
  PivotPIDTalonOutput *talonOutput_;

  //current state variables
  bool isDone_;
  bool aligning_;

  //target angle variables
  double lastJetsonAngle_, currJetsonAngle_, jetsonAngleTolerance_;
  
  //time variables
  double maxTime_, startTime_;

};
