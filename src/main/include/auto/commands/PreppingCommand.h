/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "auto/AutoCommand.h"
#include "controllers/SuperstructureController.h"


class PreppingCommand : public AutoCommand {
 public:
  PreppingCommand(RobotModel * robot, double desiredVelocity);
  PreppingCommand(RobotModel * robot);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  void Reset();
  bool IsDone();
  virtual ~PreppingCommand();
  
 private:
  bool isDone_;
  bool setVelocity_;
  RobotModel * robot_;
  double desiredVelocity_;
};
