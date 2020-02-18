/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
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
#include <frc/AnalogPotentiometer.h>
#define PI 3.141592

static const double WHEEL_DIAMETER = 0.5; //ft
//static const double HIGH_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*32/34; 
//static const double LOW_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*16/50; 
static const double HIGH_GEAR_RATIO = 44*44*44/(14*30*20.0);
static const double LOW_GEAR_RATIO = 50*44*44/(14*30*14.0);
static const double ENCODER_TICKS = 2048.0; //ticks per motor rotation
//static const double ENCODER_TICKS_FOOT = 16424.3; //might need to recheck, DONT USE
static const double HGEAR_ENCODER_TICKS_FOOT = ENCODER_TICKS*HIGH_GEAR_RATIO/(WHEEL_DIAMETER*PI); //ticks per ft
static const double LGEAR_ENCODER_TICKS_FOOT = ENCODER_TICKS*LOW_GEAR_RATIO/(WHEEL_DIAMETER*PI); // ticks per ft
//static const double MAX_HIGH_GEAR_VELOCITY = 13.3; //low gear ft/s
static const double STOP_VELOCITY_THRESHOLD = 50.0; //unit: TICKS PER SEC, threshold = 0.01 FT/SEC

static const double MAX_CURRENT_OUTPUT = 180.0; //Amps //TODO FIX
static const double MAX_DRIVE_MOTOR_CURRENT = 40.0; //Amps
//ratios work in 5 or 10% increments (accumulative) maybe? idk - medha
static const double MIN_RATIO_ALL_CURRENT = 0.2;//0.7; //TODO add to shuffleboard
static const double MIN_RATIO_SUPERSTRUCTURE_CURRENT = 0.5; //TODO add to shuffleboard
static const double MIN_RATIO_DRIVE_CURRENT = 0.7;
static const double MIN_BROWNOUT_VOLTAGE = 7.5;
static const double MAX_DRIVE_CURRENT_PERCENT = 1.0; //per motor, most teams are 40-50 Amps //TODO unused??

static double LOW_GEAR_STATIC_FRICTION_POWER = 0.0;
static double HIGH_GEAR_STATIC_FRICTION_POWER = 0.0;
static double LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER =  0.0;
static double HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER = 0.0;
static const double ROBOT_WIDTH = 28.5/12; //ft
// superstructure
static const double INTAKE_POT_OFFSET = 0.0;
static const int SPARK_ENCODER_TICKS = 42;
static const double FLYWHEEL_DIAMETER = 8.0; // inches
static constexpr auto I2CPORT = frc::I2C::Port::kOnboard;
static const double COLOR_CONFIDENCE = 0.9; // fix so it can be implemented, color matcher complaining about const double
static const int CTRE_MAG_ENCODER_TICKS = 4096;

static const int FLYWHEEL_PID_LOOP_ID = 0;
static const int FLYWHEEL_PID_TIMEOUT = 30; // milliseconds

static const double MIN_TURNING_X = 0.5;
static const double MIN_TURNING_XY_DIFFERENCE = 1.0;
static const double MAX_LOW_GEAR_VELOCITY = 8.5;

//color sensor
static constexpr frc::Color kBlueTarget = frc::Color(0.152, 0.437, 0.413);
static constexpr frc::Color kGreenTarget = frc::Color(0.193, 0.555, 0.252);
static constexpr frc::Color kRedTarget = frc::Color(0.444, 0.388, 0.171);
static constexpr frc::Color kYellowTarget = frc::Color(0.318, 0.535, 0.147);

// todo - delete later
static constexpr frc::Color BLUE = frc::Color(0.143, 0.427, 0.429); //0.127, 0.430, 0.442
static constexpr frc::Color GREEN = frc::Color(0.197, 0.561, 0.240); //0.177, 0.574, 0.249
static constexpr frc::Color RED = frc::Color(0.561, 0.232, 0.114); //0.478, 0.369, 0.153
static constexpr frc::Color YELLOW = frc::Color(0.361, 0.524, 0.113); //0.322, 0.880, 0.128


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
    

    // drive robot model
    void SetDriveValues(double left, double right);
    void SetDriveValues(RobotModel::Wheels wheels, double value);

    bool CollisionDetected();
    double GetTime();
    
    double GetLeftEncoderValue();
    double GetRightEncoderValue();
    double GetRawLeftEncoderValue();
    double GetRawRightEncoderValue();
    
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

    std::string GetAlignSequence();
    void SetAlignSequence(std::string alignSequence);

    double GetCurrentVoltage();
    double GetTotalCurrent();
    void UpdateCurrent(int channel);
    double GetCurrent(int channel);
    double ModifyCurrent(int channel, double value);
    double CheckMotorCurrentOver(int channel, double power);

    double GetTotalPower();
    double GetTotalEnergy();
    double GetCompressorCurrent();
    double GetRIOCurrent();
    double GetVoltage();

    void SetHighGear();
    void SetLowGear();
    void GearShift();

    //field error
    double SetInitLineError();
    double SetTrenchDistError(); //distance from initiation to trench edge
    double SetTrenchWidthError(); 
    double SetTrenchLengthError(); //Control Panel is too close/far
    double SetTargetZDistError();
    double SetTargetZHeightError();
    double SetLoadingDDistError();
    double SetPlayerSt2MidError(); //Player Station 2, midpoint distnance error
    double SetInitLineSlant(); //initiation line is slanted

    
    //for align tape - in drive model
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


    // superstructure robot model
    void SetAutoState(uint32_t state);
    uint32_t GetAutoState();

    double GetFlywheel1EncoderValue();
    double GetFlywheel2EncoderValue();
    void SetFlywheelOutput(double power);
    int GetFlywheelMotor1Velocity();
    void EngageFlywheelHood();
    void DisengageFlywheelHood();
    double GetTargetDistance();
    void SetControlModeVelocity(double desiredVelocity);
    void ConfigFlywheelP(double pFac_);
    void ConfigFlywheelI(double iFac_);
    void ConfigFlywheelD(double dFac_);
    void ConfigFlywheelF(double fFac_);
    
    void SetClimbWinchLeftOutput(double power);
    void SetClimbWinchRightOutput(double power);
    void SetClimberElevatorOutput(double power);

    double GetClimberWinchRightEncoderValue();
    double GetClimberWinchLeftEncoderValue(); 
    void SetIntakeWristOutput(double power);
    void SetIntakeRollersOutput(double power);
    double GetIntakeWristPotValue();
    
    double GetIntakeWristAngle();
    
    bool GetElevatorFeederLightSensorStatus();
    bool GetElevatorLightSensorStatus();

    void SetIndexFunnelOutput(double power);
    void SetElevatorFeederOutput(double power);
    void SetElevatorOutput(double power);
    
    void SetLight(bool setLight);

    void SetControlPanelOutput(double power);
    std::string GetControlPanelGameData();
    void GetColorFromSensor(); 
    std::string MatchColor();

    ~RobotModel();

  private:

    ControlBoard *humanControl_;

    frc::Timer *timer_;
    frc::PowerDistributionPanel *pdp_;
    std::string controlPanelGameData_;
    frc::Compressor *compressor_;
    frc::DoubleSolenoid *gearSolenoid_;
    frc::Solenoid *lightSolenoid_;

    AHRS *navX_;
    TalonFXSensorCollection *leftDriveEncoder_, *rightDriveEncoder_;

    NavXPIDSource* navXSource_;
    std::string testSequence_;
    std::string alignSequence_;
    WPI_TalonFX *leftMaster_, *rightMaster_, *leftSlaveA_, *rightSlaveA_;
    
    WPI_TalonFX *flywheelMotor1_, *flywheelMotor2_;
    TalonFXSensorCollection *flywheelEncoder1_, *flywheelEncoder2_; 
    frc::Solenoid *flywheelHoodSolenoid_;

    WPI_VictorSPX *climberWinchLeftMotor_, *climberWinchRightMotor_; // motor 1 - left, motor 2 - right
    WPI_TalonSRX *climberElevatorMotor_;
    frc::Encoder* climberWinchRightEncoder_, *climberWinchLeftEncoder_;
    
    //Compressor *compressor_;
    WPI_VictorSPX *controlPanelMotor_;
    rev::ColorSensorV3 *colorSensor_;
    frc::Color detectedColor_, matchedColor_;
    rev::ColorMatch colorMatcher_;
    std::string colorString_;

    WPI_VictorSPX *intakeRollersMotor_;
    WPI_TalonSRX *intakeWristMotor_;
    frc::AnalogPotentiometer *intakeWristPot_; 
    
    frc::DigitalInput *elevatorFeederLightSensor_, *elevatorLightSensor_;
    WPI_TalonSRX *indexFunnelMotor_;
    WPI_TalonSRX *elevatorMotor_, *elevatorFeederMotor_;

    double navXSpeed_;
    int counter;
    double leftDriveOutput_, rightDriveOutput_;
    double lastLeftEncoderValue_, lastRightEncoderValue_;
    double currLeftEncoderValue_, currRightEncoderValue_;
    double initialLeftEncoderValue_, initialRightEncoderValue_;
    bool isHighGear_;

    double currLeftVelocity_ , currRightVelocity_;
    double lastLeftVelocity_, lastRightVelocity_;
    float last_world_linear_accel_x_;
    float last_world_linear_accel_y_;

    double ratioAll_, ratioDrive_, ratioSuperstructure_;

    double leftDrivePower_, rightDrivePower_; // to determine the speed of the rollers (superstructure)
    double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_;
   
    uint32_t state_;
    double flywheelOneCurrent_, flywheelTwoCurrent_, climbOneCurrent_, climbTwoCurrent_;
    double intakeRollersCurrent_, intakeWristCurrent_, IndexFunnelCurrent_, elevatorFeederCurrent_, elevatorCurrent_;
    double compressorCurrent_, roboRIOCurrent_;
    bool compressorOff_, lastOver_;
    double colorConfidence_;

    double flywheelVelocTimeout_;

    double desiredDeltaAngle_;//for align tape
	  double desiredDistance_;//for align tape
    double targetVelocity_;

    double lastVelocTime_, currVelocTime_;
    double currLeftDistance_, currRightDistance_; 
    double lastLeftDistance_, lastRightDistance_; 
    // if smth is closer to the side of the opposing player station then subtract that # from the variable 
    double initLineError_ ;      
    double trenchDistError_;
    double trenchWidthError_;
    double trenchLengthError_;
    double targetZDistError_;
    double loadingDDistError_;
    double playerSt2MidError_; // add positive number if it's more to the left than expected 

    // Distance of Initiation Line To...
    double distInitLineToPS_; 
    double distInitLinetoTrench_;
    double distInitLinetoTZ_;
    double distSidewaysTZToMidTrench; // InitLineAlignedWithTZToInitLineAlignedWithMidTrench
    double distSidewaysLBToMidTrench_; //InitLineAlignedWithLBToInitLineAlignedWithMidTrench_
    double distInitLinetoCP_; // should = distInitLinetoTrench + trenchLength_        
    double distInitLineAlignedWithPSToMidTrench_;
    double distSidewaysPSToMidTrench_; 

    // Trench
    double trenchWidth_;
    double trenchLength_; // not entire trench length just side of trench near opposing alliance stations to control panel

    double distCenterLBtoCenterTZ_;
    double distSidewaysMidPSToMidTrench_; //distInitLineAlignedWithMidPSToInitLineAlignedWithMidTrench_
    

    frc::ShuffleboardTab &driverTab_, &modeTab_, &functionalityTab_, &pidTab_, &autoOffsetTab_, &superstructureTab_;
    nt::NetworkTableEntry maxOutputEntry_, minVoltEntry_, maxCurrentEntry_, leftDriveEncoderEntry_, rightDriveEncoderEntry_, leftVelocityEntry_, rightVelocityEntry_;
    nt::NetworkTableEntry lowGearSFrictionEntry_, lowGearTurnSFrictionEntry_, highGearSFrictionEntry_, highGearTurnSFrictionEntry_;
    nt::NetworkTableEntry ratioAllEntry_, ratioDriveEntry_, ratioSuperstructureEntry_;
    nt::NetworkTableEntry navXYawEntry_, voltageEntry_;


    frc::ShuffleboardLayout &driveStraightPIDLayout_, &anglePIDLayout_, &distancePIDLayout_, &pivotPIDLayout_, &curvePIDLayout_, &curveDistancePIDLayout_, &curveTurnPIDLayout_, &pointPIDLayout_;
    nt::NetworkTableEntry aPEntry_, aIEntry_, aDEntry_, dPEntry_, dIEntry_, dDEntry_, pEntry_, iEntry_, dEntry_;
    nt::NetworkTableEntry dPFacNet_, dIFacNet_, dDFacNet_; //tPFacNet_, tIFacNet_,tDFacNet_;
    nt::NetworkTableEntry pEntryP_, iEntryP_, dEntryP_;
    nt::NetworkTableEntry rColorEntry_, gColorEntry_, bColorEntry_;
    nt::NetworkTableEntry leftCurrentEntry_, rightCurrentEntry_;
    nt::NetworkTableEntry initLineErrorEntry_, trenchDistErrorEntry_, trenchWidthErrorEntry_, trenchLengthErrorEntry_, targetZDistErrorEntry_, targetZHeightErrorEntry_, loadingDDistErrorEntry_, playerSt2MidErrorEntry_, initLineSlantEntry_;
};
