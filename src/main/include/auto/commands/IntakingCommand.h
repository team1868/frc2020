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


class IntakingCommand : public AutoCommand {
 public:
  // IntakingCommand constructor 
  IntakingCommand(RobotModel * robot);

  // destructor
  virtual ~IntakingCommand();

  void Init();

  // set robot to start intaking, set isDone_ to true
  //@param currTimeSec a double
	//@param deltaTimeSec a double
  void Update(double currTimeSec, double deltaTimeSec);
  
  //@return true if done
  bool IsDone();

  void Reset();

 private:
  bool isDone_; // true if command is done
  RobotModel * robot_;
};
