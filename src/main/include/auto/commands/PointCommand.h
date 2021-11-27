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


class PointCommand : public AutoCommand {

public:
	/**
 	 * Constructor
 	 * @param robot a RobotModel
 	 * @param desiredAngle a double that is the angle of the turn
 	 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 	 * @param navXSource a NavXPIDSource
 	 * @param turnLeft a bool that represents what direction the robot should turn in. Is true if turning left
 	 * @param talonOutput a PivotPIDTalonOutput
 	 */
	PointCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource, bool turnLeft,  PivotPIDTalonOutput* talonOutput);

	/**
 	 * Constructor with tolerance
 	 * @param robot a RobotModel
 	 * @param desiredAngle a double that is the angle of the turn
 	 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 	 * @param navXSource a NavXPIDSource
 	 * @param tolerance a double
 	 * @param turnLeft a bool that represents what direction the robot should turn in
 	 * @param talonOutput a PivotPIDTalonOutput
	 */
	PointCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource, double tolerance, bool turnLeft,  PivotPIDTalonOutput* talonOutput);
	
	/**
	 * Gets PID values from init file, sets to 0 if not present
	 */
	void GetPIDValues();

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
	 * Destructor
	 */
	virtual ~PointCommand();


private:
	// pid
	double pFac_, iFac_, dFac_;
	double desiredAngle_;
	double initYaw_;
  	double relativeAngle_;

	// set to true when reset() method
	bool isDone_;

	// indicates which direction robot should turn
  	bool turnLeft_;

	int numTimesOnTarget_;

	// pointers
	RobotModel *robot_;
	frc::PIDController *pointPID_;
	NavXPIDSource * navXSource_;
	PivotPIDTalonOutput *talonOutput_;

	double maxOutput_;
	double tolerance_; 

	// minimum output to correct for, less would be considered done
	double minDrivePointOutput_;

	// time variables
	double pointCommandStartTime_;
	double pointTimeoutSec_, actualTimeoutSec_;

	// shuffle board entries
  	frc::ShuffleboardLayout &pointLayout_;
	nt::NetworkTableEntry leftDriveEntry_, rightDriveEntry_, pointErrorEntry_;
};