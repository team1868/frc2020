/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/ShootingCommand.h"

ShootingCommand::ShootingCommand(RobotModel * robot, double autoVelocity, bool setVelocity) : AutoCommand() {
    printf("shooting command\n");
    robot_ = robot;
    isDone_ = false;
    if(setVelocity){
        autoVelocity_ = autoVelocity; //this is flywheel velocity
    }
    else{
        autoVelocity_ = robot_->CalculateFlywheelVelocityDesired();
    }
    
    startShootingTime_ = 0.0;
}

void ShootingCommand::Init(){
    isDone_ = false;
    startShootingTime_ = robot_->GetTime();
    std::cout << "time starting " << startShootingTime_ << std::endl;
    robot_->SetShooting(autoVelocity_);
    robot_->ShootingAutoInit();
}

void ShootingCommand::Update(double currTimeSec, double deltaTimeSec){
    std::cout << "update :D" << std::endl;
    // if(robot_->GetTime() > robot_->GetStopDetectionTime() + 2.0){
    //     isDone_ = true;z
    // }
    isDone_ = robot_->GetShootingIsDone();
    if(isDone_){ //when shooting stops 
        printf("DONE SHOOTING IN AUTO\n");
        robot_->SetControlModeVelocity(0.0);
        //robot_->DisengageFlywheelHood();
        //robot_->SetIndexing(); //index after shoot
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
