
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/AlignMode.h"

AlignMode::AlignMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard) {
    printf("in align mode test mode constructor \n");
}

void AlignMode::CreateQueue(AutoMode::AutoPositions pos) {
    std::string alignSequence = robot_->GetAlignSequence();
    QueueFromString(alignSequence);
}

void AlignMode::Init() {
	printf("Initializing AlignMode\n");
    printf("is current command a null??? %d\n", currentCommand_==nullptr);
    //currAngle_ = robot_->GetNavXYaw();
	currentCommand_->Init();
	printf("Finished initializing\n");
}

AlignMode::~AlignMode() {

}