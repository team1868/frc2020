/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MainProgram.h"

#include <iostream>

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>

void MainProgram::RobotInit() {
    robot_ = new RobotModel();
    humanControl_ = new ControlBoard();
    superstructureController_ = new SuperstructureController(robot_, humanControl_);
    robot_->SetSuperstructureController(superstructureController_);
    driveController_ = new DriveController(robot_, humanControl_);
    robot_->CreatePIDEntries(); 
    printf("created PID entries\n"); 
    robot_->ResetDriveEncoders();
    robot_->SetHighGear();
    aligningTape_ = false;
    currJetsonAngle_ = 0.0;
    lastJetsonAngle_ = 0.0;
    jetsonAngleTolerance_ = 3.0;

    robot_->SetLastPivotAngle(robot_->GetNavXYaw());
    
    navXSource_ = new NavXPIDSource(robot_);
    talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
    
    autoSequenceEntry_ = robot_->GetModeTab().Add("Auto Test Sequence", "t 0").GetEntry();
    sequence_ = autoSequenceEntry_.GetString("t 0"); //TODO ERROR move this to auto init
    printf("I am alive.\n");


    robot_->GetDriverTab().Add("Choose auto", realAutoChooser_).WithWidget(BuiltInWidgets::kComboBoxChooser);
	realAutoChooser_.SetDefaultOption("0 blank", "n a y q n d 5.0 0");
    realAutoChooser_.AddOption("PROGRAMMING AUTO ALIGN TEST", "n a");
    realAutoChooser_.AddOption("PROGRAMMING PIVOT TEST", "n t 33.0");
	//realAutoChooser_.AddOption("1: Target Zone", "b 3545.0 s 3545.0 n t -33.0 d -8.5 0 i t 0.0 b 4812.0 d -9.5 1 n a a q n"); //Note: shooting but not making shot
	//untuned shots
    //4 degrees off last angle
    realAutoChooser_.AddOption("1: Target Zone", "n b 3370.0 s 3370.0 n t -33.0 d -8.3 0 i t 0.0 d -9.5 1 n d 11.0 0 t -20.0 a"); //Note: NO SHOT!
	realAutoChooser_.AddOption("2: Center to bar", "n a y q t -33.0 i d -7.6 0 d 6.6 0 t 0.0 a y q n");
    realAutoChooser_.AddOption("3: Shoot and move forwards", "n a y q n d 5.0 0");
    realAutoChooser_.AddOption("4: Shoot and move back", "n a y q n d -5.0 0");
    realAutoChooser_.AddOption("EXPERIMENTAL", "b 3370.0 s 3370.0 n t -33.0 d -8.3 0 i t 0.0 b 4490.0 d -9.5 1 n a s 4490.0 n"); //Note: shooting but not making shot
	//realAutoChooser_.AddOption("2: Loading Bay", "n a y q n t -118.1 d -16.53 t -53.05 d -10.0 a y q n");//d 10.0 t -38.66 d 8.93 y t 0.0 q");
	//realAutoChooser_.AddOption("3: Mid-Trench", );
	//realAutoChooser_.AddOption("4: Mid-Player Station", );
	//realAutoChooser_.AddOption("5: other", tempAutoString_);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void MainProgram::RobotPeriodic() {

    driveController_->RefreshShuffleboard();
    superstructureController_->RefreshShuffleboard();
    robot_->RefreshShuffleboard();
    
    currTime_ = robot_->GetTime();
    lastTime_ = currTime_;
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void MainProgram::AutonomousInit() {
    //TODO add test sequence sets

    //superstructureController_->SetIsAuto(true);
    robot_->SetLastPivotAngle(robot_->GetNavXYaw());
    robot_->SetHighGear();
    robot_->ResetDriveEncoders();
    robot_->ZeroNavXYaw();
    robot_->CreateNavX();
    robot_->EngageFlywheelHood();
    robot_->DisengageClimberRatchet();
    robot_->ResetWristAngle();
    robot_->SetTestSequence(robot_->GetChosenSequence());
    superstructureController_->Reset();
    superstructureController_->AutoInit();

    robot_->ZMQinit();
    //robot_->SetLight(true);
    robot_->SendZMQ(true);











    //auto selection
    robot_->SetTestSequence(realAutoChooser_.GetSelected());














    
    //robot_->SetTestSequence(sequence_);

    //start in front of target and go to trench to pick up 3 balls then shoot
    //robot_->SetTestSequence("b 3560.0 s 3560.0 n t -33.0 d -8.7 i t 0.0 d -9.5 n a y q");

    testSequence_ = new TestMode(robot_, humanControl_);
    testSequence_->QueueFromString(robot_->GetTestSequence());

    testSequence_->Init();
    std::cout<< "init time: " << robot_->GetTime() << std::endl;

    // printf("done with init, moving to periodic\n");

    //robot_->SetTestSequence("d 1.0 t 90.0 d 1.0 t 180.0 d 1.0 t -90 d 1.0 t 0.0");

    //testSequence_ = new TestMode(robot_, humanControl_);
    //testSequence_->QueueFromString(robot_->GetTestSequence());

    // printf("before init\n");
    // testSequence_->Init();

    // printf("done with init, moving to periodic\n");


    // thingS_ = new VelocityPIDSource(robot_);
    // thingO_ = new VelocityPIDOutput();
    // thingAO_ = new AnglePIDOutput();
    // thing_ = new MotionProfileTestCommand(robot_, thingS_, robot_->GetNavXSource(), thingO_, thingAO_);
    // thing_->Init();

    // tempNavXSource_ = new NavXPIDSource(robot_);
    // tempPivot_ = new PivotCommand(robot_, 90.0, true, tempNavXSource_);
    // tempPivot_->Init();
}

void MainProgram::AutonomousPeriodic() {
    robot_->UpdateZMQ();

    robot_->RefreshShuffleboard();
    // if(!tempPivot_->IsDone()){
    //     tempPivot_->Update(0.0, 0.0);
    // }
    // lastTime_ = currTime_;
    // currTime_ = robot_->GetTime();

    // //printf("AM I NULL????? %d\n", testSequence_==nullptr);

    // if(!thing_->IsDone()){
    //     thing_->Update(currTime_, currTime_-lastTime_);
    // }
    if(!testSequence_->IsDone()){
        testSequence_->Update(currTime_, currTime_-lastTime_);
    } else {
        //printf("In auto but sequence done\n");
        superstructureController_->SetIndexingState();
    }
    superstructureController_->Update(true);
}

void MainProgram::DisabledInit() {
    robot_ -> SetLight(false); //turn camera led light off end of auto
}

void MainProgram::TeleopInit() {
    //superstructureController_->SetIsAuto(false);
    robot_->SetLastPivotAngle(robot_->GetNavXYaw()); //currently unecessary

    superstructureController_->Reset();
    std::cout << "in teleopinit\n" << std::flush;
    robot_->ResetDriveEncoders();
    robot_->DisengageFlywheelHood();
    //printf("hood\n");
    robot_->DisengageClimberRatchet();
    //printf("done with climb ratchet\n");

    robot_->StartCompressor();

    matchTime_ = frc::Timer::GetMatchTime();
    aligningTape_ = false;

    std::cout << "before zmq\n" << std::flush;
    //zmq::context_t * 
    //context2_ = new zmq::context_t(1);
    robot_->ZMQinit();
    
    std::cout << "end of teleopinit\n" << std::flush;
}

void MainProgram::TeleopPeriodic() {

    robot_->UpdateZMQ();
    //bool hasContents = robot_->UpdateZMQ();

    //printf("left distance is %f and right distance is %f\n", robot_->GetLeftDistance(), robot_->GetRightDistance());
    humanControl_->ReadControls();
        //align tapes not at trench (like auto)
    //std::cout << "checking tape align\n" << std::flush;
    //TODO maybe error, setting in two locations
    if(humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton) ||
       superstructureController_->GetIsPrepping() ||
       alignTapeCommand_!=nullptr){
           
        robot_->SetLight(true);
        //printf("light on");
        robot_->SendZMQ(true);
    } else {
        robot_->SetLight(false);
        robot_->SendZMQ(false);
    }
    if (!aligningTape_ && humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
        std::cout << "READY TO START ALIGN TAPE\n" << std::endl;
        if(alignTapeCommand_!=nullptr){
            alignTapeCommand_->Reset();
            delete alignTapeCommand_;
            alignTapeCommand_ = nullptr;
            printf("deleted align tape for next iter\n");
        }
        //printf("NAVXS IS NULL? %d\n", (navXSource_==nullptr));
        alignTapeCommand_ = new AlignTapeCommand(robot_, navXSource_);
        alignTapeCommand_->Init();
        aligningTape_ = true;
        return;
    } else if (aligningTape_){
        autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX);
        autoJoyVal_ = driveController_->GetDeadbandAdjustment(autoJoyVal_);
        
        if(fabs(autoJoyVal_) >= 0.1 || alignTapeCommand_==nullptr || alignTapeCommand_->IsDone()){
            //printf("ALIGN TAPE IS NULL? %d\n", (alignTapeCommand_==nullptr));
            alignTapeCommand_->Reset();
            delete alignTapeCommand_;
            alignTapeCommand_ = nullptr;
            aligningTape_ = false;
            printf("destroyed align tape command\n");
        } else {
            //printf("ALIGN TAPE IS NULL? %d\n", (alignTapeCommand_==nullptr));
            alignTapeCommand_->Update(currTime_, currTime_-lastTime_);
            //printf("updated align tape command\n");
            return;
        }
    }

    driveController_->Update();
    //std::cout << "before superstructure\n" << std::flush;
    superstructureController_->Update(false);
    //std::cout << "updated drive and superstructure\n" << std::flush;
    //superstructureController_->WristUpdate();
    robot_->GetColorFromSensor();
    robot_->MatchColor();
    
    //std::cout << "updated colors\n" << std::flush;
    

    matchTime_ = frc::Timer::GetMatchTime();
    //sendZMQ();//sending here bc. returns after each if below and i don't want to change everything hehe

    // //trench align tapes
    // if (!aligningTape_ && humanControl_->JustPressed(ControlBoard::Buttons::kTrenchAlignButton)){
    //     aligningTape_ = true;
    //     trenchAlignTapeCommand = new TrenchAlignTapeCommand(robot_, humanControl_, navX_, talonEncoderSource_, false); //nav, talon variables don't exist yet
    //     trenchAlignTapeCommand->Init();
    //     printf("starting teleop align tapes\n");
    //     return;
    // } else if (aligningTape_){
    //     // printf("in part align tape :))\n");
    //     /*
    //     //humanControl_->ReadControls();
    //     //autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);
    //     //autoJoyVal_ = driveController_->HandleDeadband(autoJoyVal_, driveController_->GetThrustDeadband()); //TODO certain want this deadband?
    //     if(autoJoyVal_ != 0.0 || aCommand == NULL){ //TODO mild sketch, check deadbands more
    //         printf("WARNING: EXITED align.  autoJoyVal_ is %f after deadband or aCommand is NULL %d\n\n", autoJoyVal_, aCommand==NULL);
    //         delete aCommand;
    //         aCommand = NULL;
    //         aligningTape_ = false;
    //     } else */
    //     if(!trenchAlignTapeCommand->IsDone()){
    //         trenchAlignTapeCommand->Update(currTime_, currTime_-lastTime_); //(currTimeSec_, deltaTimeSec_); - variables don't exist yet
    //         printf("updated align tape command\n");
    //     } else { //isDone() is true
    //         delete trenchAlignTapeCommand;
    //         trenchAlignTapeCommand = NULL;
    //         aligningTape_ = false;
    //         printf("destroyed align tape command\n");
    //     }
    //     return;
    // }

}
/*
void MainProgram::DisabledPeriodic() {
    humanControl_->ReadControls();
    superstructureController_->DisabledUpdate();
}*/

void MainProgram::TestPeriodic() {}

void MainProgram::ResetControllers() {
    driveController_->Reset();
    superstructureController_->Reset();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MainProgram>(); }
#endif
