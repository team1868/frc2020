/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/IntakingCommand.h"

IntakingCommand::IntakingCommand(RobotModel * robot) : AutoCommand() {
    printf("intaking command\n");
    robot_ = robot;
    isDone_ = false;

}

void IntakingCommand::Init(){
    isDone_ = false;
    robot_->SetIntaking(); //true is wrist up

}

void IntakingCommand::Update(double currTimeSec, double deltaTimeSec){
    isDone_ = true;
}

bool IntakingCommand::IsDone(){
    return isDone_;
}

void IntakingCommand::Reset(){
    isDone_ = true;
}

IntakingCommand::~IntakingCommand(){
    
}
