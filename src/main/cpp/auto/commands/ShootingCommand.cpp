/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/ShootingCommand.h"

ShootingCommand::ShootingCommand(RobotModel * robot) : AutoCommand() {
    printf("shooting command\n");
    robot_ = robot;
    isDone_ = false;

}

void ShootingCommand::Init(){
    isDone_ = false;
}

void ShootingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetShooting();
    isDone_ = true;
}

bool ShootingCommand::IsDone(){
    return isDone_;
}

void ShootingCommand::Reset(){
    isDone_ = true;
}

ShootingCommand::~ShootingCommand(){
    
}
