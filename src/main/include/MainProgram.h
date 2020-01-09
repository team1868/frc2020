/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "RobotModel.h"
#include "ControlBoard.h"
#include "controllers/SuperstructureController.h"
#include "controllers/DriveController.h"
#include "auto/commands/DriveStraightCommand.h"
#include "auto/commands/PivotCommand.h"
#include "auto/commands/WaitingCommand.h"

class MainProgram : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  RobotModel *robot_;
  SuperstructureController *superstructureController_;
  DriveController *driveController_;
  ControlBoard *humanControl_;
};
