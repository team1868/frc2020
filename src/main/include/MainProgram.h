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
#include "auto/commands/AlignTapeCommand.h"
#include "auto/modes/TestMode.h"
#include "auto/commands/profiling/MotionProfileTestCommand.h"

//TODO remove this
#include "auto/PIDsource/PIDInputSource.h"

class MainProgram : public frc::TimedRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void DisabledInit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledPeriodic() override;
  void TestPeriodic() override;

  void ResetControllers();

 private:
  RobotModel *robot_;
  SuperstructureController *superstructureController_;
  DriveController *driveController_;
  ControlBoard *humanControl_;
  TalonEncoderPIDSource *talonEncoderSource_;
  NavXPIDSource *navXSource_;

  double matchTime_;

  bool aligningTape_;
  AlignTapeCommand *alignTapeCommand_;

  VelocityPIDSource *thingS_;
  VelocityPIDOutput *thingO_;
  AnglePIDOutput *thingAO_;

  TestMode *testSequence_;
  double currTime_, lastTime_;
  double autoJoyVal_;

  double lastJetsonAngle_, currJetsonAngle_, jetsonAngleTolerance_;
  
  std::string sequence_;
  nt::NetworkTableEntry autoSequenceEntry_;
  frc::SendableChooser<std::string> realAutoChooser_;
};
