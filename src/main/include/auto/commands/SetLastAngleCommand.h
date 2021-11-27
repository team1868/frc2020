// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "auto/AutoCommand.h"

class SetLastAngleCommand : public AutoCommand{
 public:
  SetLastAngleCommand(RobotModel *robot, double lastAngle);
  
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

  
  virtual ~SetLastAngleCommand();
  
 private:
  bool isDone_;
  RobotModel * robot_;
  double lastAngle_;
};
