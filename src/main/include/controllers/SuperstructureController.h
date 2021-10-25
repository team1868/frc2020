/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "ControlBoard.h"
//using namespace std;
//#include "auto/PIDSource/PIDOutputSource.h"

static const double FALCON_TO_RPM = 600.0/2048.0; //multiply to convert
#ifdef PRACTICE_BOT
static const double MAX_FALCON_RPM = 6000.0; // magic number!!!! for practice bot
static const double RATIO_BATTERY_VOLTAGE = 12.27; // for practice bot
#else
static const double MAX_FALCON_RPM = 5800.0;
static const double RATIO_BATTERY_VOLTAGE = 12.72;
#endif

class SuperstructureController {
 public:

  enum SuperstructureState {
    kControlPanel, kClimbing, kDefaultTeleop
  };

  enum PowerCellHandlingState {
    kIntaking, kIndexing, kShooting, kResetting, kUndoElevator, kManualFunnelFeederElevator
  };

  enum WristState {
    kRaising, kLowering
  }; 

  enum IndexLogicState {
    kReIndexing, kReady, kFull, kIndexingUp, kIdle
  };

  SuperstructureController(RobotModel *robot, ControlBoard *humanControl);
  void AutoInit();
  void TeleopInit();
  void Update(bool isAuto);
  void UpdatePrep(bool isAuto);
  void RefreshShuffleboard();
  void FlywheelPIDControllerUpdate();
  double CalculateFlywheelVelocityDesired();
  void SetFlywheelPowerDesired(double flywheelVelocityRPM);
  void WristUpdate(bool isAuto);
  void UpdateButtons();
  double RatioFlywheel();
  bool GetShootingIsDone();
  bool GetIsPrepping();
  
  void SetShootingState(double autoVelocity);
  void SetIntakingState();
  void SetPreppingState(double desiredVelocity);
  void SetIndexingState();
  
  void SetIsAuto(bool isAuto);

  bool IsFlywheelAtSpeed(double rpm);



  void ControlPanelStage2(double power);
  void ControlPanelStage3(double power);
  void ControlPanelFinalSpin();
  std::string GetControlPanelColor();
  void Reset();
  void Climbing();


  ~SuperstructureController();

 private:
  void IndexUpdate();
  void IndexPrep(bool isAuto);
  void Intaking();
  void Indexing();
  bool Shooting(bool isAuto);
  void Resetting();
  void UndoElevator();
  void ManualFunnelFeederElevator();
  void CheckControlPanelDesired();
  void CheckClimbDesired();
  void CheckElevatorUndoDesired();

  RobotModel *robot_;
  ControlBoard *humanControl_;
  
  SuperstructureState currSuperState_, nextSuperState_;
  PowerCellHandlingState currHandlingState_, nextHandlingState_;
  WristState currWristState_, nextWristState_;
  IndexLogicState currIndexLogicState_, nextIndexLogicState_;

  double currTime_, lastTime_;
  double startResetTime_, resetTimeout_;
  double startResetElevatorTime_;

  double flywheelResetTime_;
  double flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFac_;
  double desiredFlywheelPower_, closeFlywheelPower_;
  double autoWristDownP_, autoWristUpP_;
  double desiredFlywheelVelocity_, closeFlywheelVelocity_;

  double desiredIntakeWristAngle_;
  double currWristAngle_, lastWristAngle_;
  double intakeRollersPower_;

  double lowerElevatorTimeout_, elevatorTimeout_;
  double elevatorSlowPower_, elevatorFastPower_, elevatorFeederPower_, indexFunnelPower_, indexFunnelSlowPower_;
  double startIndexTime_, startElevatorTime_, startReIndexTime_, startIndexingTime_;
  bool bottomSensor_, topSensor_, funnelSensor_, bTimeout_, tTimeout_, resetElevatorTimeout_, jammedStartTimeout_;

  double jammedTimeout_;
  bool currJammed_;
  double motorCurrentLimit_;

  double climbElevatorUpPower_, climbElevatorDownPower_, climbPowerDesired_;

  double closeTicksPerSecDesired_;
  double farTicksPerSecDesired_;

  double shootPrepStartTime_;
  int numTimeAtSpeed_;
  bool closePrepping_, farPrepping_;
  bool atTargetSpeed_;

  int controlPanelCounter_;
  double initialControlPanelTime_;
  std::string initialControlPanelColor_, previousControlPanelColor_, colorDesired_;
  double controlPanelPower_;
  bool controlPanelStage2_, controlPanelStage3_;

  double manualRollerPower_;
  bool shootingIsDone_;

  double distanceToTarget_;
  bool isAuto_;
  bool isManualRaisingWrist_;
  double startRatchetTime_;

  // indexing logic
  bool isBallIncoming_;

  double flywheelRPMconst_;
  
  frc::ShuffleboardLayout &flywheelPIDLayout_, &sensorsLayout_, &manualOverrideLayout_, &powerLayout_, &currentLayout_, &timeoutsLayout_;
  nt::NetworkTableEntry flywheelPEntry_, flywheelIEntry_, flywheelDEntry_, flywheelFEntry_;
  //#define SUPERSTRUCTURECONTROLS
  #ifdef SUPERSTRUCTURECONTROLS
  nt::NetworkTableEntry flywheelVelocityEntry_, flywheelVelocityErrorEntry_, flywheelMotor1OutputEntry_, flywheelMotor2OutputEntry_;
  nt::NetworkTableEntry flywheelMotor1CurrentEntry_, flywheelMotor2CurrentEntry_;
  #endif

  nt::NetworkTableEntry slowElevatorEntry_, fastElevatorEntry_, funnelEntry_, rollerManualEntry_, closeFlywheelEntry_, targetSpeedEntry_;
  nt::NetworkTableEntry elevatorBottomLightSensorEntry_, elevatorTopLightSensorEntry_, funnelLightSensorEntry_;

  nt::NetworkTableEntry intakeWristAngleEntry_;
  nt::NetworkTableEntry autoWristEntry_, autoWristDownPEntry_, autoWristUpPEntry_;

  nt::NetworkTableEntry climbElevatorUpEntry_, climbElevatorDownEntry_;

  nt::NetworkTableEntry controlPanelColorEntry_;

  nt::NetworkTableEntry flywheelRPMconstEntry_;

  nt::NetworkTableEntry funnelLeftMotorEntry_, funnelRightMotorEntry_, feederMotorEntry_;

  nt::NetworkTableEntry jammedTimeoutEntry_, currentLimitEntry_;
};
