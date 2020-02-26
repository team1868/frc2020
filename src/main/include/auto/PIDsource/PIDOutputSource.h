/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include "RobotModel.h"
#include "PIDInputSource.h"

class AnglePIDOutput : public frc::PIDOutput {
public:
	/**
	 * Initializes pidOutput_ to 0
	 */
	AnglePIDOutput();
	/**
	 * Sets pidOutput_ to the output from PID loop
	 */
	void PIDWrite(double output);
	/**
	 * @return pidOutput_
	 */
	double GetPIDOutput();

	/**
	 * Destructor
	 */
	virtual ~AnglePIDOutput();
private:
	/**
	 * Output from PID loop
	 */
	double pidOutput_;
};

class DistancePIDOutput : public frc::PIDOutput {
public:
	/**
	 * Initializes pidOutput_ to 0
	 */
	DistancePIDOutput();
	/**
	 * Gets output from PID loop and assigns it to pidOutput_
	 */
	void PIDWrite(double output);
	/**
	 * @return pidOutput_
	 */
	double GetPIDOutput();
	/**
	 * Destructor
	 */
	virtual ~DistancePIDOutput();
private:
	/**
	 * Output from PID loop
	 */
	double pidOutput_;
};

/**
 * PivotPIDTalonOutput a constructor Class for WPI PIDOutput for Pivoting
 */
class PivotPIDTalonOutput : public frc::PIDOutput {

public:

	/**
	 * PivotPIDTalonOutput is a constructor that initializes robot_ and output_
	 * @param robot a RobotModel
	 */
	PivotPIDTalonOutput(RobotModel *robot);

	/**
	 * PIDWrite a function that initializes output_ and sets the drive values
	 */
	void PIDWrite(double output);

	/**
	 * PivotPIDTalonOutput is a destructor
	 */
	virtual ~PivotPIDTalonOutput();

	/**
	 * @return output
	 */
	double GetOutput();

private:
	RobotModel *robot_;
	double output_;
};

class VelocityPIDOutput : public frc::PIDOutput {

public:

	VelocityPIDOutput();

	/**
	 * PIDWrite a function that initializes output_ and sets the drive values
	 */
	void PIDWrite(double output);

	double GetPIDOutput();

	/**
	 * VelocityPIDOutput is a destructor
	 */
	virtual ~VelocityPIDOutput(); // bad thing is being destroyed so whatever

	/**
	 * @return output
	 */
	double GetOutput();

private:
	double pidOutput_;
};