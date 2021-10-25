/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/PIDSource/PIDOutputSource.h"
#include "../../../include/auto/PIDSource/PIDOutputSource.h"

/**
 * Initializes pidOutput_ to 0
 */
AnglePIDOutput::AnglePIDOutput() {
	pidOutput_ = 0.0;
}

/**
 * Sets pidOutput_ to the output from PID loop
 */
void AnglePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

/**
 * @return pidOutput_
 */
double AnglePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

/**
 * Destructor
 */
AnglePIDOutput::~AnglePIDOutput() {

}

/**
 * Initializes pidOutput_ to 0
 */
DistancePIDOutput::DistancePIDOutput() {
	pidOutput_ = 0.0;
}

/**
 * Gets output from PID loop and assigns it to pidOutput_
 */
void DistancePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

/**
 * @return pidOutput_
 */
double DistancePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

/**
 * Destructor
 */
DistancePIDOutput::~DistancePIDOutput() {

}

/**
 * sets pidOutput_ to 0.0
 */
VelocityPIDOutput::VelocityPIDOutput() {
	pidOutput_ = 0.0;
}

/**
 * PIDWrite a function that initializes output_ and sets the drive values
 */
void VelocityPIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

/**
 * @return pidOutput_
 */
double VelocityPIDOutput::GetPIDOutput() {
	return pidOutput_;
}

/**
 * VelocityPIDOutput is a destructor
 */
VelocityPIDOutput::~VelocityPIDOutput() {

}

/**
 * PivotPIDTalonOutput is a constructor that initializes robot_ and output_
 * @param robot a RobotModel
 */
PivotPIDTalonOutput::PivotPIDTalonOutput(RobotModel *robot){
	robot_ = robot;
	output_ = 0.0;
}

/**
 * PIDWrite a function that initializes output_ and sets the drive values
 */
void PivotPIDTalonOutput::PIDWrite(double myOutput){
	output_ = myOutput;
}

/**
 * @return output
 */
double PivotPIDTalonOutput::GetOutput() {
	return output_;
}
 
/**
 * Destructor
 */
PivotPIDTalonOutput::~PivotPIDTalonOutput(){
}