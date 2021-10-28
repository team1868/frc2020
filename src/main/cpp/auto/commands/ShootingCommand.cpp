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
    startShootingTime_ = 0.0;

    autoVelocity_ = autoVelocity; // this is flywheel velocity
    setVelocity_ = true;
}

ShootingCommand::ShootingCommand(RobotModel * robot) : AutoCommand() {
    printf("shooting command\n");
    robot_ = robot;
    isDone_ = false;
    startShootingTime_ = 0.0;

    autoVelocity_ = 0.0; // this is flywheel velocity
    setVelocity_ = false;
}

// gets start time, gets desired velocity (depends on constructor), sets robot to shooting state
void ShootingCommand::Init(){
    // printf("INIT SHOOTING COMMAND\n");
    // std::cout << std::endl;
    isDone_ = false;
    startShootingTime_ = robot_->GetTime();

    // if constructor doesn't have desired velocity parameter, calculate and set desired velocity
    if(!setVelocity_){
        autoVelocity_ = robot_->CalculateFlywheelVelocityDesired();
    }
    // printf("shooting command velocity: %f\n", autoVelocity_);
    // std::cout << std::endl;

    // set robot to shooting state
    robot_->SetShooting(autoVelocity_);
    robot_->ShootingAutoInit();
    // printf("INIT SHOOTING COMMAND ENDDDD\n");
    // std::cout << std::endl;
}

// checks if shooting is done
void ShootingCommand::Update(double currTimeSec, double deltaTimeSec){
    // timeout or check if done (when at desired velocity)
    // printf("SHOOTING COMMAND UPDATES\n");
    // std::cout << std::endl;
    if(robot_->GetTime()-startShootingTime_ >= 8.0){
        isDone_ = true;
    } else {
        isDone_ = robot_->GetShootingIsDone();
    }

    // when shooting stops, stop flywheel
    if(isDone_){
        robot_->SetControlModeVelocity(0.0);
    }
}

// checks if command is done
bool ShootingCommand::IsDone(){
    return isDone_;
}

// resets shooting by setting the command as finished
void ShootingCommand::Reset(){
    isDone_ = true;
}

ShootingCommand::~ShootingCommand(){
    
}
