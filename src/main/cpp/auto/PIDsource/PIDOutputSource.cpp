/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/PIDSource/PIDOutputSource.h"
#include "../../../include/auto/PIDSource/PIDOutputSource.h"

AnglePIDOutput::AnglePIDOutput() {
	pidOutput_ = 0.0;
}

void AnglePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double AnglePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

AnglePIDOutput::~AnglePIDOutput() {

}


DistancePIDOutput::DistancePIDOutput() {
	pidOutput_ = 0.0;
}

void DistancePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double DistancePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

DistancePIDOutput::~DistancePIDOutput() {

}

VelocityPIDOutput::VelocityPIDOutput() {
	pidOutput_ = 0.0;
}

void VelocityPIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double VelocityPIDOutput::GetPIDOutput() {
	return pidOutput_;
}

VelocityPIDOutput::~VelocityPIDOutput() {

}


PivotPIDTalonOutput::PivotPIDTalonOutput(RobotModel *robot){
		robot_ = robot;
		output_ = 0.0;
}

void PivotPIDTalonOutput::PIDWrite(double myOutput){
	output_ = myOutput;
}

double PivotPIDTalonOutput::GetOutput() {
	return output_;
}

PivotPIDTalonOutput::~PivotPIDTalonOutput(){
}

FlywheelPIDOutput::FlywheelPIDOutput() {
	pidOutput_ = 0.0;
}

void FlywheelPIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double FlywheelPIDOutput::GetPIDOutput() {
	return pidOutput_;
}

FlywheelPIDOutput::~FlywheelPIDOutput() {

}