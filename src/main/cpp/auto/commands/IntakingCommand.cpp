/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/IntakingCommand.h"

IntakingCommand::IntakingCommand(RobotModel * robot) : AutoCommand() {
    robot_ = robot;
    isDone_ = false;
    printf("intaking command\n");
}

// intaking command has been called so command is not done yet
void IntakingCommand::Init(){
    isDone_ = false;
}

// sets the robot's state to intaking, command is done
void IntakingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetIntaking(); // is true when wrist is up
    isDone_ = true;
}

// checks if command is done
bool IntakingCommand::IsDone(){
    return isDone_;
}

// sets command as done
void IntakingCommand::Reset(){
    isDone_ = true;
}

IntakingCommand::~IntakingCommand(){
}
