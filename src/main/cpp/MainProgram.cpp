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
    robot_->SetLastPivotAngle(robot_->GetNavXYaw());

    aligningTape_ = false;
    currJetsonAngle_ = 0.0;
    lastJetsonAngle_ = 0.0;
    jetsonAngleTolerance_ = 3.0;
    alignTapeCommand_ = nullptr;
    
    navXSource_ = new NavXPIDSource(robot_);
    talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
    
    autoSequenceEntry_ = robot_->GetModeTab().Add("Auto Test Sequence", "t 0").GetEntry();
    sequence_ = autoSequenceEntry_.GetString("t 0"); //TODO ERROR move this to auto init
    printf("I am alive.\n");

    robot_->GetDriverTab().Add("Choose auto", realAutoChooser_).WithWidget(frc::BuiltInWidgets::kComboBoxChooser);
	realAutoChooser_.SetDefaultOption("0 blank", "n a y q n d 5.0 0");
    realAutoChooser_.AddOption("PROGRAMMING AUTO ALIGN TEST", "n a");
    realAutoChooser_.AddOption("PROGRAMMING PIVOT TEST", "n t 33.0");
    realAutoChooser_.AddOption("PROGRAMMING SHOOTING TEST", "n b 3470.0 s 3470.0 n");
    realAutoChooser_.AddOption("NEW AUTO TEST", "n b 3470.0 s 3470.0 t 0.0 i d -11.7 0 d 11.7 0 a y q n");
	//realAutoChooser_.AddOption("1: Target Zone", "b 3545.0 s 3545.0 n t -33.0 d -8.5 0 i t 0.0 b 4812.0 d -9.5 1 n a a q n"); //Note: shooting but not making shot
	//untuned shots
    //4 degrees off last angle
    //lab 10ft shot is 3370
    //NASA //realAutoChooser_.AddOption("1: Target Zone", "n b 3370.0 s 3370.0 n t -33.0 d -8.3 0 i t 0.0 d -9.5 1 n d 11.0 0 t -20.0 a"); //Note: NO SHOT!
	realAutoChooser_.AddOption("1: Target Zone", "n b 3470.0 s 3470.0 n t -45.0 d -8.5 0 i t 0.0 d -9.7 1 n d 11.0 0 t -20.0 a"); //Note: NO SHOT!
	realAutoChooser_.AddOption("2: Center to bar", "n a y q t -33.0 i d -7.6 0 d 6.6 0 t 0.0 a y q n");
    realAutoChooser_.AddOption("2.: Center to bar PRACTICE MATCH TEST", "i t 0.0 d -7.6 0 d 6.6 0 t 33.0 a y q n");
    realAutoChooser_.AddOption("3: Shoot and move forwards", "n a y q n d 5.0 0");
    realAutoChooser_.AddOption("4: Shoot and move back", "n a y q n d -5.0 0");
    realAutoChooser_.AddOption("PROGRAMMING EXPERIMENTAL", "b 3470.0 s 3470.0 n t -33.0 d -8.3 0 i t 0.0 b 4490.0 d -9.5 1 n a s 4490.0 n"); //Note: shooting but not making shot
	//realAutoChooser_.AddOption("2: Loading Bay", "n a y q n t -118.1 d -16.53 t -53.05 d -10.0 a y q n");//d 10.0 t -38.66 d 8.93 y t 0.0 q");
	//realAutoChooser_.AddOption("3: Mid-Trench", );
	//realAutoChooser_.AddOption("4: Mid-Player Station", );
	//realAutoChooser_.AddOption("5: other", tempAutoString_);
    //t 26.656 before curve
    
    //working 26s sequence non sketchy
    //realAutoChooser_.AddOption("5: Slalom", "n t -40.0 d 10.5 0 t 0 d 8.0 0 t 63.435 d 7.5 0 t 26.656 c 2.75 234.274 1 1 t 125 d 6.5 0 t 180 d 11.0 0 t -142.75 d 9.362 0 t 180");
    
    //working 21s sequence
    //realAutoChooser_.AddOption("5: Slalom", "n t -40.0 d 10.5 0 t 0 d 8.7 0 t 63.435 d 7.5 0 t 26.656 c 2.75 275.0 1 1 d 4.0 0 u 180 d 11.0 0 u -142.75 d 9.0 0 t 180");
    
    //realAutoChooser_.AddOption("6: Slalom box", "n d 2.4791667 0 t 90 d -5.0 0 t 0 d 12.5 0 t -90 d -5.0 0 t 180 d -5.0 0 t 90 d -5.0 0 t 0 d -5.0 0 t -90 d -5.0 0 t 0 d -12.5 0 t 90 d -5.0 0 t 0 d -2.4791667 0");
    //11.5 battery realAutoChooser_.AddOption("Barrel", "n u -4.0 d 8.5 0 c 2.75 350.0 0 1 u -9.0 d 9.2 0 c 2.7 301 1 1 d 8.9 0 c 2.7 230 1 1 d 20.0 0");

    //realAutoChooser_.AddOption("Bounce", "n c 5.0 90 1 1 t -108.43 d -7.9 1 t 90 c 2.5 180 0 0 d -7.5 1 d 7.5 1 c 3.75 180 1 1 d 7.5 1 d -5.0 1 t 0 d 5.0 1");
    //realAutoChooser_.AddOption("Pivot Test", "n t 10.0");
    //realAutoChooser_.AddOption("90 degree test pivot", "n t 90.0");
    //realAutoChooser_.AddOption("Barrel", "n t 8.98 d 11.7421 0 t 60 c 2.75 240 0 1 t 0 d 10.098 1 c 3.5 315 1 1 t 43.84 d 8.487 1 t 18.435 c 3.16 180 1 1 t 180 d 22.5 1");
    
    //working sequences (filmed)
    //7 t
    realAutoChooser_.AddOption("Bounce no curve", "d 3.833 0 t -90 d 3.75 0 t -111.0 d -9.6 0 t 180 d -4.8 0 t 90 d -9.487 0 d 9.35 0 t 0 d 7.8 0 t -90 d 9.6 0 d -3.8 0 t 0 d 6.333 0"); //good 23 sec
    realAutoChooser_.AddOption("Bounce curve", "c 3.85 90 1 1 u -111.0 d -7.3 0 c 2.0 195.0 0 0 u 90.0 d -6.0 0 d 6.0 0 c 3.35 195.0 1 1 u -90.0 d 7.0 0 c 3.75 95.0 0 0");//d -3.5 0 t 0 d 6.333 0"); //good 23 sec
    //realAutoChooser_.AddOption("6: Slalom box", "n d 3.83 0 t -90 d 5.0 0 t 0 d 15.0 0 t 90 d 5.0 0 t 0 d 5.0 0 t -90 d 5.0 0 t 180.0 d 5.2 0 t 90 d 5.0 0 t 180.0 d 15.0 0 t -90.0 d 5.5 0 t 180.0 d 4.0 0"); //good 31 sec
    realAutoChooser_.AddOption("Barrel", "n u 0.0 d 8.0 0 c 2.75 350.0 0 1 u -9.0 d 9.0 0 c 2.5 301 1 1 d 8.7 0 c 2.6 230 1 1 d 21.0 0"); //good
    realAutoChooser_.AddOption("5: working slalom", "n u 0.0 d 3.0 0 t -65.0 d 7.0 0 t 0.0 d 10.5 0 t 65.0 d 7.4 0 t 26.0 c 2.7 265.0 1 1 u 110.0 d 5.0 0 u 180.0 d 11.5 0 u -110.0 d 9.5 0"); //good
    realAutoChooser_.AddOption("Slalom curve", "c 4.2 80.0 1 1 u 0.0 d 12.3 0 u 65.0 d 5.7 0 c 2.65 358.0 1 1 u 180.0 d 13.2 0 u -115.0 d 8.0 0");
    //4 t
    realAutoChooser_.AddOption("backwards curve test", "c 2.75 180 0 0");
    /*
    realAutoChooser_.AddOption("barrel box", "n u 0 d 11.0 0 t 90.0 d 4.5 0 t 180.0 d 4.0 0 t -90.0 d 4.167 0 t 0 d 10.0 0 t -90.0 d 4.167 0 t 180.0 d 5.0 0 t 45.0 d 11.785 0 t 0 d 2.5 0 t -90.0 d 4.167 0 t 180.0 d 20.0 0");
    realAutoChooser_.AddOption("5: Slalom test", "d 3.9 0 t -90.0 d 5.5 0 t 0 d 13.2 0 t 64.0 d 6.5 0 t 26.656 c 2.795 234.274 1 1 d 1.0 0 t 117.0 d 6.3 0 t 180 d 12.5 0 t -120.0 d 9.362 0 t 180");
    realAutoChooser_.AddOption("6: Slalom box", "d 2.4791667 0 t 90 d -5.0 0 t 0 d 12.5 0 t -90 d -5.0 0 t 180 d -5.0 0 t 90 d -5.0 0 t 0 d -5.0 0 t -90 d -5.0 0 t 0 d -12.5 0 t 90 d -5.0 0 t 0 d -2.4791667 0");
    realAutoChooser_.AddOption("Bounce", "c 60 90 1 1 t -108.43 d -7.9 0 t 90 c 30 180 0 0 d -7.5 0 d 7.5 0 c 45 180 1 1 d 7.5 0 d -5 0 t 0 d 5 0");
    realAutoChooser_.AddOption("drivestraight 5", "d 5.0 0");
    realAutoChooser_.AddOption("drivestraight 10", "d 10.0 0");
    realAutoChooser_.AddOption("drivestraight 20", "d 20.0 0");
    */

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

    robot_->SetLastPivotAngle(robot_->GetNavXYaw());
    robot_->SetHighGear();
    robot_->ResetDriveEncoders();
    robot_->ZeroNavXYaw();
    robot_->CreateNavX();
    robot_->DisengageFlywheelHood();
    robot_->DisengageClimberRatchet();
    robot_->ResetWristAngle();
    //robot_->SetTestSequence(robot_->GetChosenSequence());
    superstructureController_->Reset();
    superstructureController_->AutoInit();

    robot_->ZMQinit();
    robot_->SendZMQ(true);


    //auto selection
    robot_->SetTestSequence(realAutoChooser_.GetSelected());

    std::cout << "YOUR AUTO SEQUENCE IS " << realAutoChooser_.GetSelected() << std::endl;
 
    testSequence_ = new TestMode(robot_, humanControl_);
    testSequence_->QueueFromString(robot_->GetTestSequence());
    testSequence_->Init();

}

// updates during autonomous, checks if auto sequence is done
void MainProgram::AutonomousPeriodic() {
    
    robot_->UpdateZMQ();
    robot_->RefreshShuffleboard();

    if(!testSequence_->IsDone()){
        testSequence_->Update(currTime_, currTime_-lastTime_);
    } else {
        // In auto but sequence is done
        superstructureController_->SetIndexingState();
    }

    superstructureController_->Update(true);
}

// end of auto, turns camera led light off
void MainProgram::DisabledInit() {
    robot_ -> SetLight(false);
}

void MainProgram::TeleopInit() {

    robot_->SetLastPivotAngle(robot_->GetNavXYaw()); // currently unused but makes drive straight possible (just in case)
    superstructureController_->Reset();
    robot_->ResetDriveEncoders();
    robot_->DisengageFlywheelHood();
    robot_->DisengageClimberRatchet();
    robot_->StartCompressor();
    
    matchTime_ = frc::Timer::GetMatchTime();
    aligningTape_ = false;
    alignTapeCommand_ = nullptr;
    std::cout << "before zmq\n" << std::flush;
    robot_->ZMQinit();
    std::cout << "end of teleopinit\n" << std::flush;
}

void MainProgram::TeleopPeriodic() {

    robot_->UpdateZMQ();
    humanControl_->ReadControls();
        
    // auto align tapes
    
    if(humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton) ||
       superstructureController_->GetIsPrepping() ||
       alignTapeCommand_!=nullptr){
           
        robot_->SetLight(true);
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

        alignTapeCommand_ = new AlignTapeCommand(robot_, navXSource_);
        alignTapeCommand_->Init();
        aligningTape_ = true;
        return; // go directly into aligning (continue)

    } else if (aligningTape_){
        autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX);
        autoJoyVal_ = driveController_->GetDeadbandAdjustment(autoJoyVal_);
        
        // if the robot is moving or there is no alignTapeCommand or the command is done, destroy the command
        if(fabs(autoJoyVal_) >= 0.1 || alignTapeCommand_==nullptr || alignTapeCommand_->IsDone()){
            alignTapeCommand_->Reset();
            delete alignTapeCommand_;
            alignTapeCommand_ = nullptr;
            aligningTape_ = false;
        } else { // otherwise, update command (until done, or exits by conditions above)
            alignTapeCommand_->Update(currTime_, currTime_-lastTime_);
            return;
        }
    }

    driveController_->Update();
    superstructureController_->Update(false);
    robot_->GetColorFromSensor();
    robot_->MatchColor();
    matchTime_ = frc::Timer::GetMatchTime();

}

void MainProgram::DisabledPeriodic() {
    robot_->SetTestSequence(realAutoChooser_.GetSelected());
}

void MainProgram::TestPeriodic() {}

void MainProgram::ResetControllers() {
    driveController_->Reset();
    superstructureController_->Reset();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MainProgram>(); }
#endif
