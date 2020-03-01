/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "auto/AutoCommand.h"
#include "controllers/SuperstructureController.h"
#include "auto/commands/PivotCommand.h"

class AlignTapeCommand : public AutoCommand {
 public:
  AlignTapeCommand(RobotModel * robot);
  void Init(PivotCommand * alignTapeCommand, NavXPIDSource * navXSource);
  void Update(double currTimeSec, double deltaTimeSec, PivotCommand *alignTapeCommand);
  bool IsDone();
  void Reset();
  virtual ~AlignTapeCommand();
 private:
  bool isDone_;
  RobotModel * robot_;
  double lastJetsonAngle_, currJetsonAngle_, jetsonAngleTolerance_;
  //NavXPIDSource *navXSource_;
};
