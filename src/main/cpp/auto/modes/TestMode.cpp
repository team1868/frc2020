/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/TestMode.h"

TestMode::TestMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard) {
    printf("in test mode constructor \n");
}

void TestMode::CreateQueue(AutoMode::AutoPositions pos) {
    string testSequence = robot_->GetTestSequence();
    QueueFromString(testSequence);
}

void TestMode::Init() {
	printf("Initializing Test mode\n");
    //currAngle_ = robot_->GetNavXYaw();
	currentCommand_->Init();
	printf("Finished initializing\n");
}

TestMode::~TestMode() {

}