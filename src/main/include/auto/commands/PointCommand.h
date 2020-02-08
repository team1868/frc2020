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
	 * PivotCommand a constructor
	 * @param robot a RobotModel
	 * @param desiredAngle a double that is the angle of the turn
	 * @param isAbsolutePosition a bool that represents whether the angle is absolute position of deta angle
	 * @param navXSource a NavXPIDSource
	 */
	PointCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource);
	PointCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource, int tolerance);

	/**
	 * PivotCommand a destructor
	 */
	virtual ~PointCommand();

	/**
	 * gets Yaw from navX, sets Setpoint, continuous to false, output range, and absolute tolerance
	 */
	void Init();

	/**
	 * resets PID, sets isDone_ to true
	 */
	void Reset();

	/**
	 * if PivotPID is on target more than three times then timeout
	 * pivotPID resets, disable, isDone sets to true
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return isDone_
	 */
	bool IsDone();

	/**
	 * gets PID values from ini file, sets to 0 if not present
	 */
	void GetPIDValues();


private:
	//double output;

	double pFac_, iFac_, dFac_;
	double desiredAngle_;
	double initYaw_;
  double relativeAngle_;

	bool isDone_;
  bool turnLeft_;


	int numTimesOnTarget_;

	RobotModel *robot_;
	PIDController *pointPID_;
	NavXPIDSource * navXSource_;
	PivotPIDTalonOutput *talonOutput_;

	double maxOutput_;
	double tolerance_;

	

	/**
	 * Minimum output to correct for, less would be considered done
	 */
	double minDrivePointOutput_;

	double pointCommandStartTime_;

	double pointTimeoutSec_, actualTimeoutSec_;

  frc::ShuffleboardLayout &pointLayout_;
	nt::NetworkTableEntry leftDriveEntry_, rightDriveEntry_, pointErrorEntry_;
};