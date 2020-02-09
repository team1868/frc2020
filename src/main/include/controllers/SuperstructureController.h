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
  
  void FlywheelHoodUp();
  void FlywheelHoodDown();

  bool IndexUpdate();

  void CalculateIntakeRollersPower();

  void ControlPanelStage2(double power);
  void ControlPanelStage3(double power);
  void ControlPanelFinalSpin();
  void Reset();

  enum SuperstructureState {
		kInit, kIdle, kShooting, kIndexing, kIntaking, kControlPanelStage2, kControlPanelStage3
	};

  ~SuperstructureController();

 private:
  RobotModel *robot_;
  ControlBoard *humanControl_;
  TalonFXSensorCollection *flywheelPIDController_;
  TalonFXSensorCollection *flywheelEncoder1_, *flywheelEncoder2_; // unused rn
  
  uint32_t currState_;
	uint32_t nextState_;

  double flywheelPower_, desiredRPM_, flywheelResetTime_;
  double flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFFac_;
  PIDController *flywheelPID_;
  FlywheelPIDOutput* flywheelPIDOutput_;
  
  double flywheelFeederPower_;

  double pushNextBallTime_; // minimum time it takes for elevator to move next ball right to topmost sensor
  double elevatorPower_;
  double elevatorFeederPower_;
  double indexFunnelPower_;

  double climberPower_;

  double desiredIntakeWristAngle_;
  double currGyroAngle_, lastGyroAngle_;
  double currTime_, lastTime_;

  bool currFunnelLightSensorStatus_, currBottomElevatorLightSensorStatus_, currTopElevatorLightSensorStatus_;
  
  double numBalls_; //to keep track of how many balls inside bot
  bool isSpeed_; //is flywheel at desired speed

  int controlPanelCounter_;
  double initialControlPanelTime_;
  std::string initialControlPanelColor_, previousControlPanelColor_, colorDesired_;

  ShuffleboardLayout &superstructureLayout_;
  nt::NetworkTableEntry flywheelVelocityEntry_, flywheelPEntry_, flywheelIEntry_, flywheelDEntry_, flywheelFFEntry_;
  nt::NetworkTableEntry funnelLightSensorEntry_, bottomElevatorLightSensorEntry_, topElevatorLightSensorEntry_;
};
