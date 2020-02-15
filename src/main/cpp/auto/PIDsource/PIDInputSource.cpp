/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/PIDSource/PIDInputSource.h"
#include <frc/WPILib.h>
#include "RobotModel.h"

NavXPIDSource::NavXPIDSource(RobotModel *robot) {
	robot_ = robot;
	lastYaw_ = robot_->GetNavXYaw();
	currYaw_ = robot_->GetNavXYaw();
	accumulatedYaw_ = currYaw_;
	deltaYaw_ = currYaw_ - lastYaw_;
}

double NavXPIDSource::PIDGet() {
//	CalculateAccumulatedYaw();
	accumulatedYaw_ = robot_->GetNavXYaw();
//	accumulatedYaw_ = robot_->GetNavXYaw();
	SmartDashboard::PutNumber("Accumulated Yawwwww", accumulatedYaw_);
	return accumulatedYaw_;
}

double NavXPIDSource::CalculateAccumulatedYaw() {
	lastYaw_ = currYaw_;
	currYaw_ = robot_->GetNavXYaw();
	deltaYaw_ = currYaw_ - lastYaw_;

	if (deltaYaw_ < -180) {
		accumulatedYaw_ += (180 - lastYaw_) + (180 + currYaw_);
	} else if (deltaYaw_ > 180) {
		accumulatedYaw_ -= (180 + lastYaw_) + (180 - currYaw_);
	} else {
		accumulatedYaw_ += deltaYaw_;
	}

	return accumulatedYaw_;
}

void NavXPIDSource::ResetAccumulatedYaw() {
	accumulatedYaw_ = 0.0;
	printf("darn\n");
	currYaw_ = robot_->GetNavXYaw();
	printf("finished resetting yaw\n");
	lastYaw_ = currYaw_;
	deltaYaw_ = 0.0;
}

NavXPIDSource::~NavXPIDSource() {

}

TalonEncoderPIDSource::TalonEncoderPIDSource(RobotModel* robot) {
	robot_ = robot;
	averageTalonDistance_ = 0.0;

}

double TalonEncoderPIDSource::PIDGet() {
	double leftDistance = robot_->GetLeftDistance();
	double rightDistance = robot_->GetRightDistance();

	// FIX THIS TO BE BETTER THANKS
	if (robot_->GetLeftEncoderStopped()) {
		averageTalonDistance_ = rightDistance;
	} else if (robot_->GetRightEncoderStopped()) {
		averageTalonDistance_ = leftDistance;
	} else {
		averageTalonDistance_= (rightDistance + leftDistance) / 2;
	}

	SmartDashboard::PutNumber("Left Distance", leftDistance);
	SmartDashboard::PutNumber("Right Distance", rightDistance);
	SmartDashboard::PutNumber("Average Distance", averageTalonDistance_);
	return averageTalonDistance_;

}

TalonEncoderPIDSource::~TalonEncoderPIDSource() {

}


/*
TalonFXPIDSource::TalonFXPIDSource(RobotModel* robot) {
	robot_ = robot;
	averageTalonDistance_ = 0.0;

}

double TalonFXPIDSource::PIDGet() {
	/*
	double flywheel1Distance = robot_->GetLeftDistance();
	double flywheel2Distance = robot_->GetRightDistance();

	// FIX THIS TO BE BETTER THANKS
	if (robot_->GetLeftEncoderStopped()) {
		averageTalonDistance_ = rightDistance;
	} else if (robot_->GetRightEncoderStopped()) {
		averageTalonDistance_ = leftDistance;
	} else {
		averageTalonDistance_= (rightDistance + leftDistance) / 2;
	}

	SmartDashboard::PutNumber("Left Distance", leftDistance);
	SmartDashboard::PutNumber("Right Distance", rightDistance);
	SmartDashboard::PutNumber("Average Distance", averageTalonDistance_);
	return averageTalonDistance_;

}

TalonFXPIDSource::~TalonFXPIDSource() {

}
*/

VelocityPIDSource::VelocityPIDSource(RobotModel *robot){
	robot_ = robot;
	lastTime_ = robot_->GetTime();
	currTime_ = lastTime_;
	lastAvgPosition_ = 0.0;
	currAvgPosition_ = 0.0;
	avgVelocity_ = 0.0;
}
	
void VelocityPIDSource::UpdateVelocity(){
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();

	lastAvgPosition_ = currAvgPosition_;
	currAvgPosition_ = (robot_->GetLeftDistance()+robot_->GetRightDistance())/2.0;

	avgVelocity_ = (currAvgPosition_-lastAvgPosition_)/(currTime_-lastTime_);
}

double VelocityPIDSource::PIDGet(){
	return avgVelocity_;
}

VelocityPIDSource::~VelocityPIDSource(){

}