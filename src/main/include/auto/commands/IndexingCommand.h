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

class IndexingCommand : public AutoCommand {
 public:
  IndexingCommand(RobotModel * robot);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
  virtual ~IndexingCommand();
 private:
  bool isDone_;
  RobotModel * robot_;
};
