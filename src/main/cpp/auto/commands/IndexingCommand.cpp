/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/IndexingCommand.h"

IndexingCommand::IndexingCommand(RobotModel * robot) : AutoCommand() {
    printf("indexing command\n");
    robot_ = robot;
    isDone_ = false;

}

void IndexingCommand::Init(){
    isDone_ = false;
}

void IndexingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetIndexing();
    isDone_ = true;
}

bool IndexingCommand::IsDone(){
    return isDone_;
}

void IndexingCommand::Reset(){
    isDone_ = true;
}

IndexingCommand::~IndexingCommand(){
    
}
