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
    driveController_ = new DriveController(robot_, humanControl_);
    robot_->CreatePID();
    robot_->ResetDriveEncoders();
    robot_->CreatePID();
    printf("I am alive.");
    
    autoSequenceEntry_ = frc::Shuffleboard::GetTab("Programmer Control").Add("Auto Test Sequence", "t 0").GetEntry();
    sequence_ = autoSequenceEntry_.GetString("t 0");
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
    robot_->GetColorFromSensor();
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
    robot_->ResetDriveEncoders();
    robot_->ZeroNavXYaw();
    robot_->CreateNavX();

    //robot_->SetTestSequence("c 1.0 90.0 0");
    robot_->SetTestSequence(sequence_);

    //robot_->SetTestSequence("d 1.0 t 90.0 d 1.0 t 180.0 d 1.0 t -90 d 1.0 t 0.0");

    testSequence_ = new TestMode(robot_, humanControl_);
    testSequence_->QueueFromString(robot_->GetTestSequence());

    printf("before init\n");
    testSequence_->Init();

    // printf("done with init, moving to periodic\n");

    //robot_->SetTestSequence("d 1.0 t 90.0 d 1.0 t 180.0 d 1.0 t -90 d 1.0 t 0.0");

    // testSequence_ = new TestMode(robot_, humanControl_);
    // testSequence_->QueueFromString(robot_->GetTestSequence());

    // printf("before init\n");
    // testSequence_->Init();

    // printf("done with init, moving to periodic\n");


    // thingS_ = new VelocityPIDSource(robot_);
    // thingO_ = new VelocityPIDOutput();
    // thingAO_ = new AnglePIDOutput();
    // thing_ = new MotionProfileTestCommand(robot_, thingS_, robot_->GetNavXSource(), thingO_, thingAO_);
    // thing_->Init();

    currTime_ = robot_->GetTime();
    lastTime_ = currTime_;

    // tempNavXSource_ = new NavXPIDSource(robot_);
    // tempPivot_ = new PivotCommand(robot_, 90.0, true, tempNavXSource_);
    // tempPivot_->Init();
}

void MainProgram::AutonomousPeriodic() {
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
    }
}

void MainProgram::TeleopInit() {
    robot_->ResetDriveEncoders();
    connectZMQ();
}

void MainProgram::TeleopPeriodic() {

    //printf("left distance is %f and right distance is %f\n", robot_->GetLeftDistance(), robot_->GetRightDistance());
    humanControl_->ReadControls();
    driveController_->Update();
    superstructureController_->Update();
    robot_->GetColorFromSensor();

}

void MainProgram::DisabledPeriodic() {
    humanControl_->ReadControls();
    superstructureController_->DisabledUpdate();

}

void MainProgram::TestPeriodic() {}

void MainProgram::connectZMQ() {
    zmq::context_t * context_ = new zmq::context_t(1);

    try {
		printf("in try connect to jetson\n");
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        //change to dynamic jetson address
        subscriber_->connect("tcp://10.18.68.12:5808");
		printf("jetson connected to socket\n");
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
		printf("done with try");
        cout << endl;
    } catch(const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ALIGNWITHTAPECOMMAND INIT\n");
		std::cerr << exc.what();
	}
}

string MainProgram::readZMQ() {
    printf("starting read from jetson\n");
	string contents = s_recv(*subscriber_);
	printf("contents from jetson: %s\n", contents.c_str());
    return contents;
}

void MainProgram::readAngle(string contents) {
    
    stringstream ss(contents); //split string contents into a vector
	vector<string> result;
    bool abort_;
    double desiredDeltaAngle_;
	double desiredDistance_;

	while(ss.good()) {
		string substr;
		getline( ss, substr, ' ' );
		if (substr == "") {
			continue;
		}
		result.push_back( substr );
	}
	
	if(!contents.empty() && result.size() > 1) {
        desiredDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1));
	} else {
		abort_ = true;
		printf("contents empty in alignwithtape\n");
	}

	if(result.size() > 1) {
		desiredDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1))-1.5;//1.6;
	} else {
		abort_ = true;
		desiredDeltaAngle_ = 0.0;
		desiredDistance_ = 0.0;
	}
	printf("desired delta angle at %f in AlignWithTapeCommand\n", desiredDeltaAngle_);
    printf("desired delta distance at %f in AlignWithTapeCommand\n", desiredDistance_);
		
	/*} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cout << exc.what() << std::endl;
		desiredDeltaAngle_ = 0.0;
		// desiredDistance_ = 0.0;
	}*/

    printf("end of read angle\n");

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MainProgram>(); }
#endif
