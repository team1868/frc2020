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
  /**
   * Constructor
   * @param robot a RobotModel
   */
  IntakingCommand(RobotModel * robot);

  /**
   * Destructor
   */
  virtual ~IntakingCommand();

  /**
   * Initializes class for run
   */
  void Init();


  /** 
   * Periodic update
   * @param currTimeSec current time
   * @param deltaTimeSec delta time
   */
  void Update(double currTimeSec, double deltaTimeSec);
  
  /**
   * Returns true if command is done
   * @return isDone_
   */
  bool IsDone();

  /**
   * Resets robot to standby 
   */
  void Reset();

 private:
  bool isDone_; // true if command is done
  RobotModel * robot_;
};
