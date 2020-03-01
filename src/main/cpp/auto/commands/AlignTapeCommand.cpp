/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/AlignTapeCommand.h"

AlignTapeCommand::AlignTapeCommand(RobotModel * robot) : AutoCommand() {
    printf("AlignTapeCommand\n");
    robot_ = robot;
    isDone_ = false;
    lastJetsonAngle_ = 0.0;
    currJetsonAngle_ = 0.0;
    //autoJoyVal_ = 0.0;
    jetsonAngleTolerance_ = 3.0;
    robot_->ZMQInit();
}

void AlignTapeCommand::Init(PivotCommand * alignTapeCommand, NavXPIDSource * navXSource){
    //alignTapeCommand = new PivotCommand(robot_, robot_->GetNavXYaw()+currJetsonAngle_, true, navXSource_, 2.0);
    std::cout << "STARTING AUTO ALIGN\n" << std::flush;
    //robot_->SetLight(true);
    
    //sendZMQ(true); //tell jetson to turn exposure down

    std::string temp = robot_->ReadZMQ();
    bool hasContents = !robot_->ReadAll(temp);
    lastJetsonAngle_ = currJetsonAngle_;
    currJetsonAngle_ = robot_->GetDeltaAngle();
    printf("last jetson angle is %f and curr jetson angle is %f\n", lastJetsonAngle_, currJetsonAngle_);
    if(hasContents && fabs(lastJetsonAngle_-currJetsonAngle_) <= jetsonAngleTolerance_){
        printf("done with reading, creating aligning command");
        //bool aligningTape = true;
        if(navXSource!=nullptr){ //prevent memory leak from last run of auto align
            delete navXSource;
        }
        if(alignTapeCommand!=nullptr){
            alignTapeCommand->Reset();
            //delete alignTapeCommand;
        }
        navXSource = new NavXPIDSource(robot_); //create navX source
        printf("\nTURNING TO %f angle\n\n", currJetsonAngle_);
        printf("current angle is %f and angle to turn to is %f\n", robot_->GetNavXYaw(), robot_->GetNavXYaw()+currJetsonAngle_);
        //alignTapeCommand = new PivotCommand(robot_, robot_->GetNavXYaw()+currJetsonAngle_, true, navXSource_, 2.0);
        //alignTapeCommand = new AlignTapeCommand(robot_, humanControl_, navX_, talonEncoderSource_, false, robot_->GetDeltaAngle(), robot_->GetDistance()); //nav, talon variables don't exist yet
        printf("created aligning command");
        alignTapeCommand->Init();
        printf("starting teleop align tapes\n");
        return;
    } else {
        printf("exited jetson align, nothing read\n");
    }
        //robot_->SetLight(false);
    
    isDone_ = false;
    //robot_->SetIndexing();
}

void AlignTapeCommand::Update(double currTimeSec, double deltaTimeSec, PivotCommand *alignTapeCommand){
    //sendZMQ(true);
    // printf("in part align tape :))\n");
    //std::cout << "AM I NULL ALIGN??" << (alignTapeCommand==NULL) << std::endl << std::flush;
    if(alignTapeCommand==nullptr || alignTapeCommand->IsDone()){
        robot_->SetLight(false);
        alignTapeCommand->Reset();
        //delete alignTapeCommand;
        alignTapeCommand = NULL;
        isDone_ = true;
        printf("destroyed align tape command\n");
    } else {
        alignTapeCommand->Update(currTimeSec, deltaTimeSec);
        printf("updated align tape command\n");
        return;
    }

}

bool AlignTapeCommand::IsDone(){
    //printf("checking if done\n");
    return isDone_;
}

void AlignTapeCommand::Reset(){
    isDone_ = true;
}

AlignTapeCommand::~AlignTapeCommand(){
    
}
