// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "auto/AutoCommand.h"

class SetLastAngleCommand : public AutoCommand{
 public:
  SetLastAngleCommand(RobotModel *robot, double lastAngle);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  void Reset();
  bool IsDone();
  virtual ~SetLastAngleCommand();
  
 private:
  bool isDone_;
  RobotModel * robot_;
  double lastAngle_;
};
