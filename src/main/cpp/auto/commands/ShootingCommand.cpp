/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/ShootingCommand.h"

ShootingCommand::ShootingCommand(RobotModel * robot, double autoVelocity) : AutoCommand() {
    printf("shooting command\n");
    robot_ = robot;
    isDone_ = false;
    autoVelocity_ = autoVelocity; //this is flywheel velocity
    startShootingTime_ = 0.0;
}

void ShootingCommand::Init(){
    isDone_ = false;
    startShootingTime_ = robot_->GetTime();
    printf("time starting\n");

}

void ShootingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetShooting(autoVelocity_);
    // if(robot_->GetTime() > robot_->GetStopDetectionTime() + 2.0){
    //     isDone_ = true;z
    // }
    isDone_ = robot_->GetShootingIsDone();
    if(isDone_){ //when shooting stops 
        printf("done shooting\n");
        robot_->SetFlywheelOutput(0.0);
        robot_->SetIndexing(); //index after shoot
        //robot_->SetLight(false);
    }
}

bool ShootingCommand::IsDone(){
    return isDone_;
}

void ShootingCommand::Reset(){
    isDone_ = true;
}

ShootingCommand::~ShootingCommand(){
    
}
