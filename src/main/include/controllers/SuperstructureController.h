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
  void DisabledUpdate();
  void RefreshShuffleboard();
  void FlywheelPIDControllerUpdate();
  double CalculateFlywheelPowerDesired();
  void ControlPanelStage2(double power);
  void ControlPanelStage3(double power);
  void ControlPanelFinalSpin();
  void Reset();

  enum SuperstructureState {
		kInit, kIdle
	};

  ~SuperstructureController();

 private:
  RobotModel *robot_;
  ControlBoard *humanControl_;
  rev::CANPIDController *flywheelPIDController_;
  rev::CANEncoder *flywheelEncoder1_, *flywheelEncoder2_; // unused rn

  uint32_t currState_;
	uint32_t nextState_;

  double flywheelPower_, desiredRPM_; 
  double climberPower_;
  double flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFFac_;
  int controlPanelCounter_;
  double initialControlPanelTime_;
  std::string initialControlPanelColor_, previousControlPanelColor_, colorDesired_;

  nt::NetworkTableEntry flywheelVelocityEntry_, flywheelPEntry_, flywheelIEntry_, flywheelDEntry_, flywheelFFEntry_;
};
