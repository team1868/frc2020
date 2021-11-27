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

#define SHUFFLEBOARDCONTROLS

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

  /**
   * Constructor for SuperstructureController
   * @param robot a RobotModel
   * @param humanControl a ControlBoard
   */
  SuperstructureController(RobotModel *robot, ControlBoard *humanControl);

  // auto init
  void AutoInit();

  // teleop init
  void TeleopInit();

  void Update(bool isAuto);
  void SetIsAuto(bool isAuto);
  void Reset();

  void UpdatePrep(bool isAuto);
  void RefreshShuffleboard();
  void UpdateButtons();

  void FlywheelPIDControllerUpdate();
  double CalculateFlywheelVelocityDesired();
  void SetFlywheelPowerDesired(double flywheelVelocityRPM);
  bool IsFlywheelAtSpeed(double rpm);
  double RatioFlywheel();

  bool GetShootingIsDone();
  bool GetIsPrepping();
  void SetShootingState(double autoVelocity);
  void SetIntakingState();
  void SetPreppingState(double desiredVelocity);
  void SetIndexingState();

  void WristUpdate(bool isAuto);
  void Climbing();

  void ControlPanelStage2(double power);
  void ControlPanelStage3(double power);
  void ControlPanelFinalSpin();
  std::string GetControlPanelColor();

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
  
  // state machine
  SuperstructureState currSuperState_, nextSuperState_;
  PowerCellHandlingState currHandlingState_, nextHandlingState_;
  WristState currWristState_, nextWristState_;
  IndexLogicState currIndexLogicState_, nextIndexLogicState_;

  // timeout variables
  double currTime_, lastTime_;
  double startResetTime_, resetTimeout_;
  double startResetElevatorTime_;
  double lowerElevatorTimeout_, elevatorTimeout_;
  double startIndexTime_, startElevatorTime_, startReIndexTime_, startIndexingTime_;
  bool bottomSensor_, topSensor_, funnelSensor_, bTimeout_, tTimeout_, resetElevatorTimeout_, jammedStartTimeout_;
  double jammedTimeout_;
  double shootPrepStartTime_;
  double startRatchetTime_;

  // flywheel/shooting
  double flywheelResetTime_;
  double flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFac_;
  double desiredFlywheelPower_, closeFlywheelPower_;
  double desiredFlywheelVelocity_, closeFlywheelVelocity_;
  double closeTicksPerSecDesired_;
  double farTicksPerSecDesired_;
  int numTimeAtSpeed_;
  bool closePrepping_, farPrepping_;
  bool atTargetSpeed_;
  bool shootingIsDone_;
  double distanceToTarget_;
  double flywheelRPMconst_, highFlywheelTolerance_, flywheelPercentAdjustment_;

  // wrist angles
  double desiredIntakeWristAngle_;
  double currWristAngle_, lastWristAngle_;
  double autoWristDownP_, autoWristUpP_;
  bool isManualRaisingWrist_;

  // powers
  double intakeRollersPower_, intakeSlowRollersPower_;
  double elevatorSlowPower_, elevatorFastPower_, elevatorFeederPower_, elevatorSlowFeederPower_, indexFunnelPower_, indexFunnelSlowPower_;
  double climbElevatorUpPower_, climbElevatorDownPower_, climbPowerDesired_;
  double manualRollerPower_;
  
  // control panel
  int controlPanelCounter_;
  double initialControlPanelTime_;
  std::string initialControlPanelColor_, previousControlPanelColor_, colorDesired_;
  double controlPanelPower_;
  bool controlPanelStage2_, controlPanelStage3_;

  bool isAuto_;
  bool currJammed_;
  double motorCurrentLimit_;

  // indexing logic
  bool isBallIncoming_;


  frc::ShuffleboardLayout &flywheelPIDLayout_, &sensorsLayout_, &manualOverrideLayout_, &powerLayout_, &currentLayout_, &timeoutsLayout_;
  nt::NetworkTableEntry flywheelPEntry_, flywheelIEntry_, flywheelDEntry_, flywheelFEntry_;
  
  
  #ifdef SHUFFLEBOARDCONTROLS
  nt::NetworkTableEntry flywheelVelocityEntry_, flywheelVelocityErrorEntry_, flywheelMotor1OutputEntry_, flywheelMotor2OutputEntry_;
  nt::NetworkTableEntry flywheelMotor1CurrentEntry_, flywheelMotor2CurrentEntry_;
  #endif

  nt::NetworkTableEntry slowElevatorEntry_, fastElevatorEntry_, funnelEntry_, rollerManualEntry_, closeFlywheelEntry_, targetSpeedEntry_;
  nt::NetworkTableEntry elevatorBottomLightSensorEntry_, elevatorTopLightSensorEntry_, funnelLightSensorEntry_;
  nt::NetworkTableEntry intakeWristAngleEntry_;
  nt::NetworkTableEntry autoWristEntry_, autoWristDownPEntry_, autoWristUpPEntry_;
  nt::NetworkTableEntry climbElevatorUpEntry_, climbElevatorDownEntry_;
  nt::NetworkTableEntry controlPanelColorEntry_;
  nt::NetworkTableEntry flywheelRPMconstEntry_, highFlywheelToleranceEntry_, flywheelPercentAdjustmentEntry_;
  nt::NetworkTableEntry funnelLeftMotorEntry_, funnelRightMotorEntry_, feederMotorEntry_;
  nt::NetworkTableEntry jammedTimeoutEntry_, currentLimitEntry_;

};
