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
  /**
	 * ShootingCommand a constructor
	 * @param robot a RobotModel
	 * @param autoVelocity a double that is flywheel velocity
	 */
  ShootingCommand(RobotModel * robot, double autoVelocity);
  
  /**
	 * ShootingCommand a constructor
	 * @param robot a RobotModel
	 */
  ShootingCommand(RobotModel * robot);

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

  /**
   * Destructor
   */ 
  virtual ~ShootingCommand();
  
 private:
  bool isDone_;
  RobotModel * robot_;

  // shooting variables
  bool setVelocity_;
  double autoVelocity_;
  double startShootingTime_;
};
