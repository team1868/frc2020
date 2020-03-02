/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "auto/AutoCommand.h"
#include "controllers/SuperstructureController.h"

class ShootingCommand : public AutoCommand{
 public:
  ShootingCommand(RobotModel * robot, double autoVelocity);
  ShootingCommand(RobotModel * robot);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
  virtual ~ShootingCommand();
 private:
  bool isDone_;
  RobotModel * robot_;
  bool setVelocity_;
  double autoVelocity_;
  double startShootingTime_;
};
