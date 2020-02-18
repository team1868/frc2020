/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "ControlBoard.h"
//#include "auto/PIDSource/PIDOutputSource.h"

class SuperstructureController {
 public:
  /*
  enum SuperstructureState {
		kInit, kShooting, kIndexing, kIntaking, kResetting, 
    kControlPanelStage2, kControlPanelStage3, kClimbingElevator,
    kClimbing
	};
  */

  enum SuperstructureState {
    kControlPanel, kClimbing, kDefaultTeleop
  };

  enum ClimbingState {
    kClimbingIdle, kClimbingElevator, kClimbingWinches
  };

  enum PowerCellHandlingState {
    kIntaking, kIndexing, kShooting, kResetting
  };

  enum AutoState {
    kAutoInit, kAutoCloseShooting, kAutoFarShooting,
    kAutoIntaking, kAutoIndexing
  };

   enum WristState {
    kRaising, kLowering
  }; 

  SuperstructureController(RobotModel *robot, ControlBoard *humanControl);
  void Update();
  void AutoUpdate();
  void DisabledUpdate();
  void RefreshShuffleboard();
  void FlywheelPIDControllerUpdate();
  double CalculateFlywheelVelocityDesired();
  void SetFlywheelPowerDesired(double flywheelVelocity);
  void WristUpdate();
  void WinchUpdate();
  
  //these functions should not exist
  void FlywheelHoodUp();
  void FlywheelHoodDown();

  bool IsFlywheelAtSpeed();

  bool IndexUpdate();

  double CalculateIntakeRollersPower();


  void ControlPanelStage2(double power);
  void ControlPanelStage3(double power);
  void ControlPanelFinalSpin();
  void Reset();



  ~SuperstructureController();

 private:
  RobotModel *robot_;
  ControlBoard *humanControl_;
  
  uint32_t currAutoState_, nextAutoState_;
  uint32_t currState_, nextState_;
  ClimbingState currClimbingState_;
  PowerCellHandlingState currHandlingState_, nextHandlingState_;
  WristState currWristState_, nextWristState_;

  double currTime_, lastTime_;
  double startResetTime_, resetTimeout_;

  double flywheelResetTime_;
  double flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFac_;
  double desiredFlywheelPower_, closeFlywheelPower_;
  double desiredFlywheelVelocity_, closeFlywheelVelocity_;
  PIDController *flywheelPID_;
  SuperstructurePIDOutput *flywheelPIDOutput_;
  TalonFXPIDSource *flywheelPIDSource_;

  double wristPFac_;
  double desiredIntakeWristAngle_;
  double currWristAngle_, lastWristAngle_;

  double lowerElevatorTimeout_, elevatorTimeout_;
  double elevatorSlowPower_, elevatorFastPower_, elevatorFeederPower_, indexFunnelPower_;
  double startIndexTime_, startElevatorTime_;
  bool bottomSensor_, topSensor_, bTimeout_, tTimeout_;

  double climbElevatorUpPower_, climbElevatorDownPower_, climbPowerDesired_;
  bool positiveDirection_;
  double climbWinchPower_, climbWinchUpdatePower_;
  double currRobotAngle_;

  double closeTicksPerSecDesired_;
  double farTicksPerSecDesired_;

  double shootPrepStartTime_;
  bool closePrepping_, farPrepping_;

  int controlPanelCounter_;
  double initialControlPanelTime_;
  std::string initialControlPanelColor_, previousControlPanelColor_, colorDesired_;
  double controlPanelPower_;
  bool controlPanelStage2_, controlPanelStage3_;


  ShuffleboardLayout &flywheelPIDLayout_, &sensorsLayout_, &manualOverrideLayout_;
  nt::NetworkTableEntry flywheelPEntry_, flywheelIEntry_, flywheelDEntry_, flywheelFEntry_;
  nt::NetworkTableEntry flywheelVelocityEntry_, flywheelVelocityErrorEntry_;

  nt::NetworkTableEntry wristPEntry_;
  nt::NetworkTableEntry intakeWristAngleEntry_;
  nt::NetworkTableEntry winchAutoEntry_;
  nt::NetworkTableEntry elevatorBottomLightSensorEntry_, elevatorTopLightSensorEntry_;
};
