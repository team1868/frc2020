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
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "auto/PIDsource/PIDInputSource.h"
#include "auto/PIDsource/PIDOutputSource.h"
#include "Ports2020.h"
#define PI 3.141592

static const double WHEEL_DIAMETER = 4.0 / 12.0; //ft
static const double HIGH_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*32/34; //ft INCORRECT
static const double LOW_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*16/50; //INCORRECT
//static const double ENCODER_TICKS = 2048.0; //units per rotation
static const double ENCODER_TICKS_FOOT = 16424.3;
static const double MAX_HIGH_GEAR_VELOCITY = 13.3; //low gear ft/s
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

class RobotModel {
  public:
    enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
    RobotModel();
    frc::ShuffleboardTab& GetDriverTab();
    frc::ShuffleboardTab& GetModeTab();
    frc::ShuffleboardTab& GetFunctionalityTab();
    frc::ShuffleboardTab& GetPIDTab();
    frc::ShuffleboardTab& GetAutoOffsetTab();
    void SetDriveValues(double left, double right);
    void SetDriveValues(RobotModel::Wheels wheels, double value);

    bool CollisionDetected();
    double GetTime();
    
    double GetLeftEncoderValue();
    double GetRightEncoderValue();
    
    void ResetDriveEncoders();
    void RefreshShuffleboard();

     void StartCompressor();
    
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

    double GetTotalPower();
    double GetTotalEnergy();
    double GetCompressorCurrent();
    double GetRIOCurrent();
    double GetVoltage();

    // PID Stuff (yay)
    void CreatePID(); 

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

    void DriveStraightPIDUpdate();

    ~RobotModel();

  private:
    frc::Timer *timer_;
    frc::PowerDistributionPanel *pdp_;
    frc::Compressor *compressor_;
    AHRS *navX_;
    TalonFXSensorCollection *leftDriveEncoder_, *rightDriveEncoder_;

    double navXSpeed_;
    int counter;
    double leftDriveOutput_, rightDriveOutput_;
    double lastLeftEncoderValue_, lastRightEncoderValue_;
    double currLeftEncoderValue_, currRightEncoderValue_;

    double currLeftVelocity_ , currRightVelocity_;
    double lastLeftVelocity_, lastRightVelocity_;
    float last_world_linear_accel_x_;
    float last_world_linear_accel_y_;

    double ratioAll_, ratioDrive_, ratioSuperstructure_;
    double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_;
    double compressorCurrent_, roboRIOCurrent_;
    bool compressorOff_, lastOver_;
    
    NavXPIDSource* navXSource_;
    std::string testSequence_;
    WPI_TalonFX *leftMaster_, *rightMaster_, *leftSlaveA_, *rightSlaveA_;

    frc::ShuffleboardTab &driverTab_, &modeTab_, &functionalityTab_, &pidTab_, &autoOffsetTab_;
    nt::NetworkTableEntry maxOutputEntry_, minVoltEntry_, maxCurrentEntry_, leftDriveEncoderEntry_, rightDriveEncoderEntry_, leftVelocityEntry_, rightVelocityEntry_;
    nt::NetworkTableEntry lowGearSFrictionEntry_, lowGearTurnSFrictionEntry_, highGearSFrictionEntry_, highGearTurnSFrictionEntry_;
    nt::NetworkTableEntry ratioAllEntry_, ratioDriveEntry_, ratioSuperstructureEntry_;
    nt::NetworkTableEntry navXYawEntry_, voltageEntry_;

    frc::ShuffleboardLayout &driveStraightPIDLayout_, &anglePIDLayout_, &distancePIDLayout_, &pivotPIDLayout_, &curvePIDLayout_, &curveDistancePIDLayout_, &curveTurnPIDLayout_;
    nt::NetworkTableEntry aPEntry_, aIEntry_, aDEntry_, dPEntry_, dIEntry_, dDEntry_, pEntry_, iEntry_, dEntry_;
    nt::NetworkTableEntry dPFacNet_, dIFacNet_, dDFacNet_, tPFacNet_, tIFacNet_,tDFacNet_;

};
