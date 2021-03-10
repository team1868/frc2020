/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/WaitingCommand.h"

/**
 * Assigns the waitTimeSec and creates the timer
 */
WaitingCommand::WaitingCommand(RobotModel *robot, double myWaitTimeSec) : AutoCommand() {
	waitTimeSec_ = myWaitTimeSec;
	robot_ = robot;
	startTime_ = -1.0;
	isDone_ = false;
}

// starts timer
void WaitingCommand::Init() {
	startTime_ = robot_->GetTime();
}

/**
 * Checks if the timer meets the waitTimeSec. If so, isDone is set to true.
 */
void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	if(startTime_ > 0.0){
		isDone_ = currTimeSec - startTime_ >= waitTimeSec_;
		if(isDone_) {
			printf("done waiting %f", currTimeSec);
		}
	} else {
		Init();
 	}

}

/**
 * @return isDone
 */
bool WaitingCommand::IsDone() {
	return isDone_;
}

// resets isDone_ to true
void WaitingCommand::Reset() {
	isDone_ = true;
}

// destructor
WaitingCommand::~WaitingCommand() {
}