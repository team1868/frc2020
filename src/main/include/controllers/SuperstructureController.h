/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "ControlBoard.h"

class SuperstructureController {
 public:
  SuperstructureController(RobotModel *robot, ControlBoard *humanControl);
  void Update();
  void RefreshShuffleboard();
  ~SuperstructureController();
 private:
  RobotModel *robot_;
  ControlBoard *humanControl_;
};
