/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <AHRS.h>
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "RobotModel.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>

/**
 * A class implementing Pivot PID the WPILibrary PID Controller
 */
class PivotCommand : public AutoCommand {

public:
	/** 
     * Constructor without tolerance
     * @param robot a RobotModel
 	 * @param desiredAngle a double that is the angle of the turn
 	 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of delta angle
 	 * @param navXSource a NavXPIDSource
 	 * @param talonOutput a PivotPIDTalonOutput
 	 */
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource, PivotPIDTalonOutput* talonOutput);
	
	/** 
 	 * Constructor with tolerance
 	 * @param robot a RobotModel
	 * @param desiredAngle a double that is the angle of the turn
 	 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 	 * @param navXSource a NavXPIDSource
 	 * @param tolerance a double
 	 * @param talonOutput a PivotPIDTalonOutput
 	 */
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource, int tolerance, PivotPIDTalonOutput* talonOutput);

	/**
	 * Destructor
	 */
	virtual ~PivotCommand();

	/**
 	 * Initializes class for run
 	 */
	void Init();

	/**
     * Resets robot to standby 
     */
	void Reset();

    /** 
     * Periodic update
     * @param currTimeSec current time
     * @param deltaTimeSec delta time
     */
	void Update(double currTimeSec, double deltaTimeSec);

    /**
     * Returns true if command is done
     * @return isDone_
     */
	bool IsDone();

	/**
	 * gets PID values from Shuffleboard, sets to 0 if not present
	 */
	void GetPIDValues();

private:
	double pFac_, iFac_, dFac_;
	double desiredAngle_;
	double initYaw_;

	bool isDone_;

	int numTimesOnTarget_;

	RobotModel *robot_;
	frc::PIDController *pivotPID_;
	NavXPIDSource * navXSource_;
	PivotPIDTalonOutput *talonOutput_;

	double maxOutput_;
	double tolerance_;

	double pivotCommandStartTime_;
	double pivotTimeoutSec_;

  	frc::ShuffleboardLayout &pivotLayout_;
	nt::NetworkTableEntry leftDriveEntry_, rightDriveEntry_, pivotErrorEntry_;
};
