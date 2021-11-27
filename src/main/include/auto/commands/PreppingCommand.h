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
  /**
   * Constructor
   * @param robot a RobotModel
   * @param desiredVelocity a double
   */ 
  PreppingCommand(RobotModel * robot, double desiredVelocity);
  
  /**
   * Constructor without desired velocity
   * @param robot a RobotModel
   */ 
  PreppingCommand(RobotModel * robot);

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
    * Resets robot to standby 
    */
  void Reset();

  /**
   * Returns true if command is done
   * @return isDone_
   */
  bool IsDone();

  /**
   * Destructor
   */ 
  virtual ~PreppingCommand();
  
 private:
  bool isDone_;
  bool setVelocity_;
  RobotModel * robot_;
  double desiredVelocity_;
};
