/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <math.h>
#include "RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>

class DriveStraightCommand : public AutoCommand {
  public:
    /** 
     * Constuctor with slow option
     * @param navXSource a NavXPIDSource
     * @param talonEncoderSource a TalonEncoderPIDSource
     * @param distancePIDOutput an AnglePIDOutput
     * @param distancePIDOutput a DistancePIDOutput
     * @param robot a RobotModel
     * @param desiredDistance a double that refers to the desired distance to travel
     * @param slow is true if robot should move slowly
     */
    DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
        AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
        double desiredDistance, bool slow);
    
    /** 
     * Constructor with absolute angle option
     * @param navXSource a NavXPIDSource
     * @param talonEncoderSource a TalonEncoderPIDSource
     * @param anglePIDOutput an AnglePIDOutput
     * @param distancePIDOutput a DistancePIDOutput
     * @param robot a RobotModel
     * @param desiredDistance a double that refers to the desired distance to travel
     * @param slow is true if robot should move slowly
     * @param absoluteAngle a double that refers to the current absolute angle (keep the same angle while driving straight)
     */
    DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
          AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
          double desiredDistance, bool slow, double absoluteAngle);

    /**
     * Destructor
     */
    virtual ~DriveStraightCommand();

    /**
     * Initializes class for run
     */
    void Init();

    /** 
     * Periodic update
     * @param currTimeSec current time
     * @param deltaTimeSec delta time
     */
    void Update(double currTime, double deltaTime);

    /**
     * Returns true if done (repeatedly on target)
     * @returns isDone_
     */ 
    bool IsDone();

    /**
     *  Resets robot to standby
     */ 
    void Reset();

    /**
     * Gets PID values from shuffleboard
     */
    void GetPIDValues();

private:

  /**
    * Initializes dependencies
    * @param navXSource a NavXPIDSource
    * @param talonEncoderSource a TalonEncoderPIDSource
    * @param anglePIDoutput an AnglePIDOutput
    * @param distancePIDOutput a DistancePIDOutput
    * @param robot a RobotModel
    * @param desiredDistance a double that refers to the desired distance to travel
    */
	void Initializations(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance);

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *anglePIDOutput_;
	DistancePIDOutput *distancePIDOutput_;
	frc::PIDController *anglePID_;
	frc::PIDController *distancePID_;
	RobotModel *robot_;

	bool isAbsoluteAngle_;

  // PID dependencies
	double rPFac_, rIFac_, rDFac_;
	double rMaxOutput_, rTolerance_;
	double dPFac_, dIFac_, dDFac_;
	double dMaxOutput_, dTolerance_;
  double initialDMax_, finalDMax_, maxT_;
	double desiredAngle_;
	double initialAvgDistance_;
	double desiredDistance_;
	double desiredTotalAvgDistance_;

  // motor output
	double leftMotorOutput_, rightMotorOutput_;

  // to check if done
	double initialDriveTime_, diffDriveTime_, driveTimeoutSec_;
	bool isDone_;
	int numTimesOnTarget_;
	double lastDOutput_;
	double lastDistance_;
	int numTimesStopped_;

  // slow
  bool slow_;
  double slowSpeed_;
  
  // shuffleboard
  frc::ShuffleboardLayout &driveStraightLayout_;
	nt::NetworkTableEntry leftStraightEntry_, rightStraightEntry_, angleErrorEntry_, angleErrorGraphEntry_, desiredAngleEntry_,
	  encoderErrorEntry_, encoderErrorGraphEntry_, desiredTotalFeetEntry_, dPIDOutputEntry_, aPIDOutputEntry_;
};
