/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/WaitingCommand.h"

WaitingCommand::WaitingCommand(RobotModel *robot, double myWaitTimeSec) : AutoCommand() {
	waitTimeSec_ = myWaitTimeSec;
	robot_ = robot;
	startTime_ = -1.0;
	// timer_ = new frc::Timer();
	isDone_ = false;
}

void WaitingCommand::Init() {
	startTime_ = robot_->GetTime();
	//timer_->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	// isDone_ = (timer_->Get() >= waitTimeSec_);
	// std::cout << isDone_ << waitTimeSec_ << std::endl;
	isDone_ = currTimeSec - startTime_ >= waitTimeSec_;
	if(isDone_) {
		printf("done waiting %f", currTimeSec);
	}
}

bool WaitingCommand::IsDone() {
	return isDone_;
}

void WaitingCommand::Reset() {
}

WaitingCommand::~WaitingCommand() {
}