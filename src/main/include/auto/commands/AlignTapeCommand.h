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

class AlignTapeCommand : public AutoCommand{
 public:
  AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource_);
  ~AlignTapeCommand();
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
 private:
  RobotModel *robot_;
  PivotCommand *pivotCommand_;
  NavXPIDSource *navXSource_;
  bool isDone_;
  bool aligning_;
  double lastJetsonAngle_, currJetsonAngle_, jetsonAngleTolerance_;
  double maxTime_, startTime_;
};
