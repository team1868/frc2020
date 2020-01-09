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

static const double WHEEL_DIAMETER = 6.0 / 12.0; //ft
static const double HIGH_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*0.0; //ft INCORRECT
static const double LOW_GEAR_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*0.0; //INCORRECT
static const double ENCODER_TICKS = 2000.0; //TODO INCORRECT
static const double MAX_HIGH_GEAR_VELOCITY = 13.3; //low gear ft/s

static const double MAX_CURRENT_OUTPUT = 180.0; //Amps //TODO FIX
static const double MAX_DRIVE_MOTOR_CURRENT = 40.0; //Amps
static const double MIN_RATIO_DRIVE_CURRENT = 0.7;
static const double MIN_VOLTAGE_BROWNOUT = 7.5;
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
    void ZeroNavXYaw();
    double GetNavXYaw();
    double GetNavXPitch();
    double GetNavXRoll();
    double GetRightDistance();
    double GetLeftDistance();
    bool GetLeftEncoderStopped();
    bool GetRightEncoderStopped();
    std::string GetTestSequence();
    void CreateNavX();
    NavXPIDSource* GetNavXSource();
    ~RobotModel();
  private:
    frc::Timer *timer_;
    frc::PowerDistributionPanel *pdp_;
    AHRS *navX_;
    frc::Encoder *leftDriveEncoder_, *rightDriveEncoder_;
    double navXSpeed_;
    double leftDriveOutput_, rightDriveOutput_;
    float last_world_linear_accel_x_;
    float last_world_linear_accel_y_;
    NavXPIDSource* navXSource_;
    std::string testSequence_;
    WPI_TalonSRX *leftMaster_, *rightMaster_;
    WPI_VictorSPX *leftSlaveA_, *leftSlaveB_, *leftSlaveC_, *rightSlaveA_, *rightSlaveB_, *rightSlaveC_;
    frc::ShuffleboardTab &driverTab_, &modeTab_, &functionalityTab_, &pidTab_, &autoOffsetTab_;
    nt::NetworkTableEntry maxOutputEntry_, minVoltEntry_, maxCurrentEntry_;
    nt::NetworkTableEntry lowGearSFrictionEntry_, lowGearTurnSFrictionEntry_, highGearSFrictionEntry_, highGearTurnSFrictionEntry_;
};
