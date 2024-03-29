/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#define PRACTICE_BOT

#include <zhelpers.hpp>
#include <frc/WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "auto/PIDsource/PIDInputSource.h"
#include "auto/PIDsource/PIDOutputSource.h"
#include "Ports2020.h"
#include "ControlBoard.h"
#include "controllers/SuperstructureController.h"
#include <math.h>
#define PI 3.141592

static const double WHEEL_DIAMETER = 0.5; //ft
static const double HIGH_GEAR_RATIO = 44*44*44/(14*30*20.0);
static const double LOW_GEAR_RATIO = 50*44*44/(14*30*14.0);
static const double ENCODER_TICKS = 2048.0; // ticks per motor rotation
static const double HGEAR_ENCODER_TICKS_FOOT = ENCODER_TICKS*HIGH_GEAR_RATIO/(WHEEL_DIAMETER*PI); // ticks per ft
static const double LGEAR_ENCODER_TICKS_FOOT = ENCODER_TICKS*LOW_GEAR_RATIO/(WHEEL_DIAMETER*PI); // ticks per ft
static const double STOP_VELOCITY_THRESHOLD = 0.01; // unit: TICKS PER SEC, threshold = 0.01 FT/SEC

static const double MAX_CURRENT_OUTPUT = 180.0; // Amps
static const double MAX_DRIVE_MOTOR_CURRENT = 40.0; // Amps
static const double MIN_RATIO_ALL_CURRENT = 0.2;
static const double MIN_RATIO_SUPERSTRUCTURE_CURRENT = 0.5; 
static const double MIN_RATIO_DRIVE_CURRENT = 0.7;
static const double MIN_BROWNOUT_VOLTAGE = 7.5;
static const double MAX_DRIVE_CURRENT_PERCENT = 1.0; // per motor, most teams are 40-50 Amps 

static double LOW_GEAR_STATIC_FRICTION_POWER = 0.0;
static double HIGH_GEAR_STATIC_FRICTION_POWER = 0.0;
static double LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER =  0.0;
static double HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER = 0.0;
static const double ROBOT_WIDTH = 28.5/12; //f t

// superstructure
static const int SPARK_ENCODER_TICKS = 42;
static const double FLYWHEEL_DIAMETER = 8.0; // inches
static constexpr auto I2CPORT = frc::I2C::Port::kOnboard;
static const double COLOR_CONFIDENCE = 0.9; // fix so it can be implemented, color matcher complaining about const double
static const int CTRE_MAG_ENCODER_TICKS = 4096;

static const int FLYWHEEL_PID_LOOP_ID = 0;
static const int FLYWHEEL_PID_TIMEOUT = 30; // milliseconds

static const double MIN_TURNING_X = 0.5;
static const double MIN_TURNING_XY_DIFFERENCE = 1.0;
static const double MAX_LOW_GEAR_VELOCITY = 7.5;
static const double TICKS_TO_WRIST_DEGREES = 360.0/4096*18/34; // 0.04653

//color sensor
static constexpr frc::Color kBlueTarget = frc::Color(0.152, 0.437, 0.413);
static constexpr frc::Color kGreenTarget = frc::Color(0.193, 0.555, 0.252);
static constexpr frc::Color kRedTarget = frc::Color(0.444, 0.388, 0.171);
static constexpr frc::Color kYellowTarget = frc::Color(0.318, 0.535, 0.147);

class SuperstructureController;
class RobotModel {
  public:
    enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
    RobotModel();
    frc::ShuffleboardTab& GetDriverTab();
    frc::ShuffleboardTab& GetModeTab();
    frc::ShuffleboardTab& GetFunctionalityTab();
    frc::ShuffleboardTab& GetPIDTab();
    frc::ShuffleboardTab& GetAutoOffsetTab();
    frc::ShuffleboardTab& GetSuperstructureTab();

    void SetSuperstructureController(SuperstructureController *superstructureControllers);
    
    void ShootingAutoInit();

    // drive robot model
    void SetDriveValues(double left, double right);
    void SetDriveValues(RobotModel::Wheels wheels, double value);

    bool CollisionDetected();
    double GetTime();
    
    double GetLeftEncoderValue();
    double GetRightEncoderValue();
    double GetRawLeftEncoderValue();
    double GetRawRightEncoderValue();

    void SetIndexing();
    void SetIntaking();
    void SetShooting(double autoVelocity);
    void SetPrepping(double desiredVelocity);

    bool GetShootingIsDone();
    
    void ResetDriveEncoders();
    void RefreshShuffleboard();

    void StartCompressor();
    double GetPressureSwitchValue();
    
    void ZeroNavXYaw();
    double GetNavXYaw();
    double GetNavXPitch();
    double GetNavXRoll();
    void CreateNavX();
    NavXPIDSource* GetNavXSource();
    
    double GetRightDistance();
    double GetLeftDistance();
    double GetRightVelocity();
    double GetLeftVelocity();
    bool GetLeftEncoderStopped();
    bool GetRightEncoderStopped();
    std::string GetTestSequence();
    void SetTestSequence(std::string testSequence);

    double GetCurrentVoltage();
    double GetTotalCurrent();
    void UpdateCurrent(int channel);
    double GetCurrent(int channel);
    double ModifyCurrent(int channel, double value);
    double CheckMotorCurrentOver(int channel, double power);

    double CalculateFlywheelVelocityDesired();

    double GetTotalPower();
    double GetTotalEnergy();
    double GetCompressorCurrent();
    double GetRIOCurrent();
    double GetVoltage();

    double GetFeederMotorStatus();
    double GetLeftFunnelMotorStatus();
    double GetRightFunnelMotorStatus();

    void SetHighGear();
    void SetLowGear();
    void GearShift();
    bool IsHighGear();

    // field error
    double SetInitLineError();
    double SetTrenchDistError(); // distance from initiation to trench edge
    double SetTrenchWidthError(); 
    double SetTrenchLengthError(); // Control Panel is too close/far
    double SetTargetZDistError();
    double SetTargetZHeightError();
    double SetLoadingDDistError();
    double SetPlayerSt2MidError(); // Player Station 2, midpoint distnance error
    double SetInitLineSlant(); // initiation line is slanted

    void CheckAllianceColor();
    std::string GetDefaultSequence();
    std::string GetChosenSequence();
    std::string GetChosenSequence1();
    std::string GetChosenSequence2();
    std::string GetChosenSequence3();
    std::string GetChosenSequence4();
    
    // for align tape - in drive model
    void SetDeltaAngle(double angle);
    void SetDistance(double distance);
    double GetDeltaAngle();
    double GetDistance();

    // PID Stuff (yay)
    void CreatePIDEntries(); 

    double GetDriveStraightAngleP();
    double GetDriveStraightAngleI();
    double GetDriveStraightAngleD();
    double GetDriveStraightDistanceP();
    double GetDriveStraightDistanceI();
    double GetDriveStraightDistanceD();

    double GetPivotP();
    double GetPivotI();
    double GetPivotD();

    double GetCurveDistanceP();
    double GetCurveDistanceI();
    double GetCurveDistanceD();
    double GetCurveTurnP();
    double GetCurveTurnI();
    double GetCurveTurnD();

    double GetPointP();
    double GetPointI();
    double GetPointD();

    void ResetWristAngle();

    // superstructure robot model
    void SetAutoState(uint32_t state);
    uint32_t GetAutoState();

    double GetFlywheel1EncoderValue();
    double GetFlywheel2EncoderValue();
    void SetFlywheelOutput(double power);
    int GetFlywheelMotor1Velocity();
    void EngageFlywheelHood();
    void DisengageFlywheelHood();
    void EngageClimberRatchet();
    void DisengageClimberRatchet();
    void SetControlModeVelocity(double desiredVelocity);
    void ConfigFlywheelF(double fFac_);
    void ConfigFlywheelPID(double pFac_, double iFac_, double dFac_);
    double FlywheelMotor1Output();
    double FlywheelMotor2Output();
    bool IsAutoFlywheelAtSpeed(double desiredVelocity);

    double GetFlywheelMotor1Current();
    double GetFlywheelMotor2Current();
    
    void SetRightClimberElevatorOutput(double power);
    void SetLeftClimberElevatorOutput(double power);

    void SetIntakeWristOutput(double power);
    void SetIntakeRollersOutput(double power);
    
    double GetIntakeWristAngle();
    
    bool GetElevatorFeederLightSensorStatus();
    bool GetElevatorLightSensorStatus();
    bool GetFunnelLightSensorStatus();

    void SetIndexFunnelOutput(double power);
    void SetElevatorFeederOutput(double power);
    void SetElevatorOutput(double power);
    
    void SetLight(bool setLight);

    void SetControlPanelOutput(double power);
    std::string GetControlPanelGameData();
    void GetColorFromSensor(); 
    std::string MatchColor();

    void SetLastPivotAngle(double value);
    double GetLastPivotAngle();

    // zmq
    void ConnectRecvZMQ();
    std::string ReadZMQ();
    bool ReadAll(std::string contents);
    void ConnectSendZMQ();
    void SendZMQ(bool lowExposure);
    void ZMQinit();
    void UpdateZMQ();
    bool ZMQHasContents();

    ~RobotModel();

  private:

    ControlBoard *humanControl_;
    SuperstructureController *superstructureController_;

    frc::Timer *timer_;
    frc::PowerDistributionPanel *pdp_;
    std::string controlPanelGameData_;
    frc::Compressor *compressor_;
    frc::Solenoid *gearSolenoid_;
    frc::Solenoid *lightSolenoid_;

    AHRS *navX_;
    TalonFXSensorCollection *leftDriveEncoder_, *rightDriveEncoder_;

    NavXPIDSource* navXSource_;
    std::string testSequence_;
    std::string alignSequence_;
    WPI_TalonFX *leftMaster_, *rightMaster_, *leftSlaveA_, *rightSlaveA_;
    
    // superstructure
    StatorCurrentLimitConfiguration *fortyAmpFXLimit_;
    SupplyCurrentLimitConfiguration *thirtyAmpSRXLimit_, *fortyAmpSRXLimit_;

    WPI_TalonFX *flywheelMotor1_, *flywheelMotor2_;
    TalonFXSensorCollection *flywheelEncoder1_, *flywheelEncoder2_; 
    frc::Solenoid *flywheelHoodSolenoid_;

    WPI_TalonSRX *climberRightElevatorMotor_, *climberLeftElevatorMotor_;
    frc::DigitalInput *limitSwitchRight_, *limitSwitchLeft_;
    frc::Solenoid *climberRatchetSolenoid_;
    
    WPI_VictorSPX *controlPanelMotor_;
    rev::ColorSensorV3 *colorSensor_;
    frc::Color detectedColor_, matchedColor_;
    rev::ColorMatch colorMatcher_;
    std::string colorString_;

    WPI_VictorSPX *intakeRollersMotor_;
    WPI_TalonSRX *intakeWristMotor_;
    
    frc::DigitalInput *elevatorFeederLightSensor_, *elevatorLightSensor_, *funnelLightSensor_;

#ifdef PRACTICE_BOT
    WPI_TalonSRX *indexFunnelMotor_, *elevatorFeederMotor_; // practice bot
#else
    WPI_VictorSPX *indexFunnelMotorA_, *indexFunnelMotorB_, *elevatorFeederMotor_; // comp bot
#endif
    WPI_VictorSPX *elevatorMotor_;

    double navXSpeed_;
    int counter;
    double leftDriveOutput_, rightDriveOutput_;
    double lastLeftEncoderValue_, lastRightEncoderValue_;
    double currLeftEncoderValue_, currRightEncoderValue_;
    double initialLeftEncoderValue_, initialRightEncoderValue_;
    bool isHighGear_;
    bool resetWristAngle_;

    double currLeftVelocity_ , currRightVelocity_;
    double lastLeftVelocity_, lastRightVelocity_;
    float last_world_linear_accel_x_;
    float last_world_linear_accel_y_;

    double ratioAll_, ratioDrive_, ratioSuperstructure_;

    double leftDrivePower_, rightDrivePower_; // to determine the speed of the rollers (superstructure)
    double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_;
   
    uint32_t state_;
    int numTimeAtSpeed_;
    double flywheelOneCurrent_, flywheelTwoCurrent_, climbOneCurrent_, climbTwoCurrent_;
    // note: index funnel currents are not being used currently
    double intakeRollersCurrent_, intakeWristCurrent_, indexFunnelACurrent_, indexFunnelBCurrent_, elevatorFeederCurrent_, elevatorCurrent_;
    double compressorCurrent_, roboRIOCurrent_;
    bool compressorOff_, lastOver_;
    double colorConfidence_;

    double flywheelVelocTimeout_;

    double desiredDeltaAngle_; // for align tape
	  double desiredDistance_; // for align tape
    double targetVelocity_;

    double lastVelocTime_, currVelocTime_;
    double currLeftDistance_, currRightDistance_; 
    double lastLeftDistance_, lastRightDistance_; 

    double lastPivotAngle_;
    
    // choosing strings
    std::string chosenSequence_;
    std::string testSequence1_;
    std::string testSequence2_;
    std::string testSequence3_;
    std::string testSequence4_;

    // if smth is closer to the side of the opposing player station then subtract that # from the variable
    double initLineError_ ;
    double trenchDistError_;
    double trenchWidthError_;
    double trenchLengthError_;
    double targetZDistError_; // + if TZ is wider than supposed to be
    double loadingDDistError_; // + if LD is wider than supposed to be
    double playerSt2MidError_; // add positive number if it's more to the left than expected aka closer to LD

    // Distance of Initiation Line To...
    double distInitLineToPS_;
    double distInitLinetoTrench_;
    double distInitLinetoTZ_;
    double distInitLinetoLB_;
    double distSidewaysTZToMidTrench; // InitLineAlignedWithTZToInitLineAlignedWithMidTrench
    double distSidewaysLBToMidTrench_; //InitLineAlignedWithLBToInitLineAlignedWithMidTrench_
    double distInitLinetoCP_; // should = distInitLinetoTrench + trenchLength_
    double distInitLineAlignedWithPSToMidTrench_;
    double distSidewaysPSToMidTrench_;

    // Trench
    double trenchWidth_;
    double trenchLength_; // not entire trench length just side of trench near opposing alliance stations to control panel

    double distCenterLBtoCenterTZ_;
    double distSidewaysMidPSToMidTrench_;
    double distSidewaysTZToMidTrench_;
    double distMidPSToMidTZ_;

    // Auto Sequence Variables (in the strings)
    std::string strTrenchLength_;

    // Sequence 1
    double angleA1_,distA1_,distB1_;
    std::string strAngleA1_,strDistA1_,strDistB1_;

    //Sequence 2
    double angleA2_,angleB2_,angleC2_,distA2_,distB2_;
    std::string strAngleA2_,strAngleB2_,strAngleC2_,strDistA2_,strDistB2_;

    // Sequence 3
    double angleA3_,distA3_;
    std::string strAngleA3_,strDistA3_;

    // Sequence 4
    double angleA4_,angleB4_,angleC4_,distA4_,distB4_;
    std::string strAngleA4_,strAngleB4_,strAngleC4_,strDistA4_,strDistB4_;

    // input sequence
    std::string autoInputSequence_;

    
    //zmq
    zmq::context_t *context_; // context for creating sockets
    zmq::socket_t *subscriber_; // socket to receive message from jetson
    zmq::socket_t *publisher_; // socket to send message to jetson
    int confl;
    bool isSocketBound_;
    bool hasContents_;

    frc::ShuffleboardTab &driverTab_, &modeTab_, &functionalityTab_, &pidTab_, &autoOffsetTab_, &superstructureTab_;
    nt::NetworkTableEntry maxOutputEntry_, minVoltEntry_, maxCurrentEntry_, leftDriveEncoderEntry_, rightDriveEncoderEntry_, leftVelocityEntry_, rightVelocityEntry_;
    nt::NetworkTableEntry lowGearSFrictionEntry_, lowGearTurnSFrictionEntry_, highGearSFrictionEntry_, highGearTurnSFrictionEntry_;
    nt::NetworkTableEntry ratioAllEntry_, ratioDriveEntry_, ratioSuperstructureEntry_;
    nt::NetworkTableEntry navXYawEntry_, voltageEntry_;
    nt::NetworkTableEntry climberRightLimitSwitchEntry_, climberLeftLimitSwitchEntry_; 

    frc::ShuffleboardLayout &driveStraightPIDLayout_, &anglePIDLayout_, &distancePIDLayout_, &pivotPIDLayout_, &curvePIDLayout_, &curveDistancePIDLayout_, &curveTurnPIDLayout_, &pointPIDLayout_;
    nt::NetworkTableEntry aPEntry_, aIEntry_, aDEntry_, dPEntry_, dIEntry_, dDEntry_, pEntry_, iEntry_, dEntry_;
    nt::NetworkTableEntry dPFacNet_, dIFacNet_, dDFacNet_;
    nt::NetworkTableEntry pEntryP_, iEntryP_, dEntryP_;
    nt::NetworkTableEntry rColorEntry_, gColorEntry_, bColorEntry_;
    nt::NetworkTableEntry resetWristAngleEntry_;
    nt::NetworkTableEntry leftCurrentEntry_, rightCurrentEntry_;
    nt::NetworkTableEntry initLineErrorEntry_, trenchDistErrorEntry_, trenchWidthErrorEntry_, trenchLengthErrorEntry_, targetZDistErrorEntry_, targetZHeightErrorEntry_, loadingDDistErrorEntry_, playerSt2MidErrorEntry_, initLineSlantEntry_;
    nt::NetworkTableEntry autoSequenceEntry_;
    frc::SendableChooser<std::string> autoSendableChooser_;

};
