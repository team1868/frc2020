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

    isSocketBound_ = false;
    
    autoSequenceEntry_ = robot_->GetModeTab().Add("Auto Test Sequence", "t 0").GetEntry();
    sequence_ = autoSequenceEntry_.GetString("t 0"); //TODO ERROR move this to auto init
    printf("I am alive.\n");

    //context_ = new zmq::context_t(2); //same context for send + receive zmq
    //publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
    //subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
    //connectRecvZMQ();
    //connectSendZMQ();
    context_ = nullptr; //same context for send + receive zmq
    publisher_ = nullptr;
    subscriber_ = nullptr;
    printf("done with zmq init\n");
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
    robot_->SetHighGear();
    robot_->ResetDriveEncoders();
    robot_->ZeroNavXYaw();
    robot_->CreateNavX();
    robot_->SetTestSequence(robot_->GetChosenSequence());
    superstructureController_->Reset();

    //zmq
    if (context_ == nullptr) {
        context_ = new zmq::context_t(2); //same context for send + receive zmq
        publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        ConnectRecvZMQ();
        ConnectSendZMQ();
    }

    //robot_->SetTestSequence("c 1.0 90.0 0");
    //robot_->SetTestSequence(sequence_);

    //robot_->SetTestSequence("d 1.0 c 3.0 180.0 0"); //for testing high gear and low gear
    //robot_->SetTestSequence("c 3.0 90.0 0 0");
    robot_->SetTestSequence("i w 4.0 b 3560.0 s 3560.0 n");// c 4.0 90.0 1 1");
    
    //robot_->SetTestSequence("d 1.0 t 90.0 d 1.0 t 180.0 d 1.0 t -90.0 d 1.0 t 0.0"); //for testing high gear and low gear

    navXSource_ = new NavXPIDSource(robot_);
    talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

    //robot_->SetTestSequence("d 1.0 t 90.0 d 1.0 t 180.0 d 1.0 t -90 d 1.0 t 0.0");

    testSequence_ = new TestMode(robot_, humanControl_);
    testSequence_->QueueFromString(robot_->GetTestSequence());


    //robot_->SetLight(true); //turn on light for auto

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
        printf("In auto but sequence done\n");
        superstructureController_->SetIndexingState();
    }
    superstructureController_->Update(true);
}

void MainProgram::DisabledInit() {
    robot_ -> SetLight(false); //turn camera led light off end of auto
}

void MainProgram::TeleopInit() {
    //superstructureController_->SetIsAuto(false);
    superstructureController_->Reset();
    std::cout << "in teleopinit\n" << std::flush;
    robot_->ResetDriveEncoders();

    robot_->StartCompressor();

    matchTime_ = frc::Timer::GetMatchTime();
    aligningTape_ = false;

    std::cout << "before zmq\n" << std::flush;
    //zmq::context_t * 
    //context2_ = new zmq::context_t(1);
    
    if (context_ == nullptr) {
        context_ = new zmq::context_t(2); //same context for send + receive zmq
        publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        ConnectRecvZMQ();
        ConnectSendZMQ();
    }
    std::cout << "end of teleopinit\n" << std::flush;
}

void MainProgram::TeleopPeriodic() {

    //printf("left distance is %f and right distance is %f\n", robot_->GetLeftDistance(), robot_->GetRightDistance());
    humanControl_->ReadControls();
        //align tapes not at trench (like auto)
    //std::cout << "checking tape align\n" << std::flush;
    if(humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
        robot_->SetLight(true);
        //printf("light on");
        SendZMQ(true);
    } else {
        robot_->SetLight(false);
        SendZMQ(false);
    }
    if (!aligningTape_ && humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
        std::cout << "READY TO START ZMQ READ\n" << std::flush;
        //robot_->SetLight(true);
        
        //sendZMQ(true); //tell jetson to turn exposure down

        std::string temp = ReadZMQ();
        bool hasContents = !ReadAll(temp);
        lastJetsonAngle_ = currJetsonAngle_;
        currJetsonAngle_ = robot_->GetDeltaAngle();
        printf("last jetson angle is %f and curr jetson angle is %f\n", lastJetsonAngle_, currJetsonAngle_);
        if(hasContents && fabs(lastJetsonAngle_-currJetsonAngle_) <= jetsonAngleTolerance_){
            printf("done with reading, creating aligning command");
            aligningTape_ = true;
            if(navXSource_!=nullptr){ //prevent memory leak from last run of auto align
                delete navXSource_;
            }
            if(alignTapeCommand!=nullptr){
                delete alignTapeCommand;
            }
            navXSource_ = new NavXPIDSource(robot_); //create navX source
            printf("\nTURNING TO %f angle\n\n", currJetsonAngle_);
            printf("current angle is %f and angle to turn to is %f\n", robot_->GetNavXYaw(), robot_->GetNavXYaw()-currJetsonAngle_);
            alignTapeCommand = new PivotCommand(robot_, robot_->GetNavXYaw()-currJetsonAngle_, true, navXSource_, 2.0);
            //alignTapeCommand = new AlignTapeCommand(robot_, humanControl_, navX_, talonEncoderSource_, false, robot_->GetDeltaAngle(), robot_->GetDistance()); //nav, talon variables don't exist yet
            printf("created aligning command");
            alignTapeCommand->Init();
            printf("starting teleop align tapes\n");
            return;
        } else {
            printf("exited jetson align, nothing read\n");
        }
        //robot_->SetLight(false);
    } else if (aligningTape_){
        //sendZMQ(true);
        // printf("in part align tape :))\n");
        autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);
        autoJoyVal_ = driveController_->GetDeadbandAdjustment(autoJoyVal_);
        //std::cout << "AM I NULL ALIGN??" << (alignTapeCommand==NULL) << std::endl << std::flush;
        if(fabs(autoJoyVal_) >= 0.1 || alignTapeCommand==nullptr || alignTapeCommand->IsDone()){
            robot_->SetLight(false);
            delete alignTapeCommand;
            alignTapeCommand = NULL;
            aligningTape_ = false;
            printf("destroyed align tape command\n");
        } else {
            alignTapeCommand->Update(currTime_, currTime_-lastTime_);
            printf("updated align tape command\n");
            return;

        }
    }// else {
       // sendZMQ(false);
    //}

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

void MainProgram::ConnectRecvZMQ() {
    //connect to zmq socket to receive from jetson
    try {
		printf("in try connect to jetson\n");
        //change to dynamic jetson address
        subscriber_->connect("tcp://10.18.68.12:5808");
		printf("jetson connected to socket\n");
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000); //TODO THIS MIGHT ERROR
        subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0); //filter for nothing
    } catch(const zmq::error_t &exc) {
		printf("ERROR: TRY CATCH FAILED IN ZMQ CONNECT RECEIVE\n");
		std::cerr << exc.what();
	}
    std::cout << "reached end of connect recv zmq\n" << std::flush;
}

std::string MainProgram::ReadZMQ() {
    /*try {
		printf("in try connect to jetson in readZMQ\n");
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        //change to dynamic jetson address
        subscriber_->connect("tcp://10.18.68.12:5808");
		printf("jetson connected to socket\n");
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000); //TODO THIS MIGHT ERROR
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0); //filter for nothing
    } catch(const zmq::error_t &exc) {
		printf("ERROR: TRY CATCH FAILED IN ZMQ CONNECT RECEIVE\n");
		std::cerr << exc.what();
	}
    */
    printf("starting read from jetson\n");
	//std::string contents = s_recv(*subscriber_);
	std::string contents = s_recv(*subscriber_);
    return contents;
}

// void MainProgram::readDistance(string contents){

// }

// void MainProgram::readDetected(string contents){

// }

bool MainProgram::ReadAll(std::string contents) {
    printf("ready to read from jetson\n");
    
    std::stringstream ss(contents); //split string contents into a vector
	std::vector<std::string> result;
    bool abort;

	while(ss.good()) {
		std::string substr;
		getline( ss, substr, ' ' );
		if (substr == "") {
			continue;
		}
		result.push_back( substr );
	}
	
	// if(!contents.empty() && result.size() > 1) {
    //     robot_->SetDeltaAngle( stod(result.at(0)) );
	// 	robot_->SetDistance( stod(result.at(1)) );
    //     abort = false;
	// } else {
	// 	abort = true;
	// 	printf("contents empty in alignwithtape\n");
	// }

    //jetson string is hasTarget, angle (deg from center), raw distance (ft)
	if(result.size() > 2) {
        printf("received values\n"); //TODO MAYBE ERROR CHECK ZMQ SEND ON JETSON SIDE
        double angle = stod(result.at(1));
        double distance = stod(result.at(2));
        printf("jetson set angle is %f and set distance is %f\n", angle, distance);
		robot_->SetDeltaAngle(angle);
		robot_->SetDistance(distance);//1.6;
        abort = false;
	} else {
		abort = true;
		robot_->SetDeltaAngle(0.0);
		robot_->SetDistance(0.0);
        printf("returning because nothing received\n");
	}
	printf("desired delta angle at %f in AlignWithTapeCommand\n", robot_->GetDeltaAngle());
    printf("desired delta distance at %f in AlignWithTapeCommand\n", robot_->GetDistance());
		
	/*} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cout << exc.what() << std::endl;
		desiredDeltaAngle_ = 0.0;
		// desiredDistance_ = 0.0;
	}*/

    printf("end of read angle with %d\n", abort);
    return abort;

}

void MainProgram::ConnectSendZMQ() {
    //zmq socket to send message to jetson
    try{
        std::cout << "start connect send zmq\n" << std::flush;
        //std::cout << "done connect socket zmq\n" << std::flush;
        if(!isSocketBound_){
            publisher_->bind("tcp://*:5807");
            isSocketBound_ = true;
        }
        int confl = 1;
        std::cout << "setting socket zmq\n" << std::flush;
        publisher_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
        std::cout << "done setting socket zmq\n" << std::flush;
    } catch (const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ZMQ CONNECT SEND\n");
		std::cerr << exc.what();
	}

}

void MainProgram::SendZMQ(bool lowExposure) {
    //string message = "matchtime = " + to_string(matchTime_) + ", aligningTape = " + to_string(aligningTape_);
    std::string message = std::to_string(lowExposure);
    //std::cout << message << std::endl;
    //zmq_send((void *)publisher_, message.c_str(), message.size(), 0);
    int sent = zmq_send((void *)*publisher_, message.c_str(), message.size(), 0);
    //std::cout << sent << " done sending to zmq" << std::endl;
}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MainProgram>(); }
#endif
