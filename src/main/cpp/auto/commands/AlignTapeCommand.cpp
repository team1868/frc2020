/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/AlignTapeCommand.h"

AlignTapeCommand::AlignTapeCommand(RobotModel* robot, ControlBoard* humanControl, NavXPIDSource* navXSource, TalonEncoderPIDSource* talonSource, bool driveStraightDesired) : AutoCommand() {
    printf("constructing align with tape command\n");
	//this is align tape command for auto for shooting on the target side of trench. robot does not back up before pivot.
    
    context_ = NULL;
    subscriber_ = NULL;

    robot_ = robot;
	humanControl_ = humanControl;

    navXSource_ = navXSource;
    talonSource_ = talonSource;

    driveStraightDesired_ = driveStraightDesired;

    anglePIDOutput_ = new AnglePIDOutput();
    distancePIDOutput_ = new DistancePIDOutput();

    pivotCommand_ = NULL;
    driveStraightCommand_ = NULL;
	alignSequence_ = new AlignMode(robot_, humanControl_);

    isDone_ = false;
	abort_ = false;

    desiredDeltaAngle_ = robot_->GetDeltaAngle();
    desiredDistance_ = robot_->GetDistance();

    currState_ = kInit;
    nextState_ = kInit;

    initTimeVision_ = 0.0;
    initTimeAlign_ = 0.0;
}

void AlignTapeCommand::Init() {
    printf("in align with tape command init\n");

    robot_->SetLight(true); //turn on light for auto align

	initTimeAlign_ = robot_->GetTime();
	initTimeVision_ = robot_->GetTime(); //TODO is this right

    context_ = new zmq::context_t(1);
    /* zmq connect done in main program
    try {
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        subscriber_->connect("tcp://10.18.68.12:5808");
		printf("connected to socket\n");
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
    } catch(const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ALIGNWITHTAPECOMMAND INIT\n");
		std::cerr << exc.what();
	}*/

    desiredDeltaAngle_ = 0.0;
    desiredDistance_ = 0.0;
    isDone_ = false;
	abort_ = false;
    currState_ = kInit;
	nextState_ = kInit;

	anglePIDOutput_ = new AnglePIDOutput();
	distancePIDOutput_ = new DistancePIDOutput();

	initTimeVision_ = robot_->GetTime();
}

void AlignTapeCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("in align with tape update()");
    double lastDesiredAngle = desiredDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;

	desiredDeltaAngle_ = robot_->GetDeltaAngle(); //update from zmq constant read in mainprogram
    desiredDistance_ = robot_->GetDistance(); //update from zmq constant read in mainprogram

	double diffInAngle;

	if (abort_) {
		isDone_ = true;
		nextState_ = kInit;
	}

	printf("Vision pivot delta angle %f\n", desiredDeltaAngle_);
	printf("Vision desired distance %f\n", desiredDistance_);
	diffInAngle = fabs(lastDesiredAngle - desiredDeltaAngle_);
	printf("Difference in Angle: %f\n", diffInAngle);

	//setting align sequence based on angle
	if (fabs(desiredDeltaAngle_) > 2.0) {
		//pivot robot if angle change needed is more than 2 degrees
		printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);
		printf("ANGLE turning FOR PIVOT COMMAND: %f, abs angle turning to is %f (includes orig angle)\n", desiredDeltaAngle_, robot_->GetNavXYaw()+desiredDeltaAngle_);

		string angle = to_string(desiredDeltaAngle_);
		stringSequence_ = "t " + angle;
		//robot_->SetAlignSequence(stringSequence_);
		
	} else {
		//don't need to pivot
		stringSequence_ = "d 0";
		//robot_->SetAlignSequence(stringSequence_);

		printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);
		printf("ANGLE WAS GOOD, NO PIVOT: %f\n", -desiredDeltaAngle_);
		if(driveStraightDesired_) {
			printf("driving now");
			nextState_ = kUpdate;
		} else {
			printf("leaving");
			isDone_ = true;
		}
	}
	robot_->SetAlignSequence(stringSequence_);

	switch (currState_) {
		case (kInit):
			printf("In kInit\n");
			//ReadFromJetson();
			if (abort_) {
				isDone_ = true;
				nextState_ = kInit;
				break;
			}
			//alignSequence_ = new AlignMode(robot_, humanControl_);
    		alignSequence_->QueueFromString(robot_->GetAlignSequence());
			alignSequence_->Init();

			printf("align sequence inited: %f\n", robot_->GetTime() - initTimeVision_);
			break;

		case (kUpdate):
			printf("in kUpdate");
			if (!alignSequence_->IsDone()) {
				alignSequence_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kUpdate;
				printf("in alignwithtape update() command on, updated command\n");
			} else {
				//ReadFromJetson();
				if (abort_) {
					isDone_ = true;
					nextState_ = kInit;
					break;
				}
				printf("Final Vision Angle: %f\n", desiredDeltaAngle_);
				printf("Pivot To Angle Is Done\n");
				if (driveStraightDesired_) {
					nextState_ = kInit;
				} else {
					isDone_ = true;
					printf("AlighWithTape Done \n");
				}
			}
			break;
	} 
	

	/*
	switch (currState_) {
		case (kPivotInit) :
			printf("In kPivotInit\n");

			//ReadFromJetson();
			if (abort_) {
				isDone_ = true;
				nextState_ = kPivotInit;
				break;
			}
			printf("Vision pivot delta angle %f\n", desiredDeltaAngle_);
			printf("Vision desired distance %f\n", desiredDistance_);

			diffInAngle = fabs(lastDesiredAngle - desiredDeltaAngle_);
			printf("Difference in Angle: %f\n", diffInAngle);
			// if (fabs(diffInAngle) < 2.0 || (diffInAngle == 0.0 && desiredDeltaAngle_ == 0.0)) {
			// 	printf("diff in angle is < 2\n");
			// 	nextState_ = kPivotInit;
			// } else 
			if (fabs(desiredDeltaAngle_) > 2.0) {
				//pivot robot if angle change needed is more than 2 degrees
				printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);

				printf("ANGLE turning FOR PIVOT COMMAND: %f, abs angle turning to is %f (includes orig angle)\n", desiredDeltaAngle_, robot_->GetNavXYaw()+desiredDeltaAngle_);
				pivotCommand_ = new PivotCommand(robot_, robot_->GetNavXYaw()+desiredDeltaAngle_, true, navXSource_, 1); //last digit is pivot tolerance
				printf("pivotCommand constructed at time: %f\n", robot_->GetTime() - initTimeVision_);
				pivotCommand_->Init();
				printf("pivotCommand inited: %f\n", robot_->GetTime() - initTimeVision_);
				nextState_ = kPivotUpdate;
			} else {
				printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);
				printf("ANGLE WAS GOOD, NO PIVOT: %f\n", -desiredDeltaAngle_);
				if(driveStraightDesired_) {
					printf("driving now");
					nextState_ = kDriveInit;
				} else {
					printf("leaving align update kpivotinit");
					isDone_ = true;
				}
			}
			break;

		case (kPivotUpdate):
			printf("in kPivot update");
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotUpdate;
				printf("in alignwithtape update() pivot command on, updated pivot command\n");
			} else {
				//ReadFromJetson();
				if (abort_) {
					isDone_ = true;
					nextState_ = kPivotInit;
					break;
				}
				printf("Final Vision Angle: %f\n", desiredDeltaAngle_);
				printf("Pivot To Angle Is Done\n");
				if (driveStraightDesired_) {
					nextState_ = kDriveInit;
				} else {
					isDone_ = true;
					printf("AlighWithTape Done \n");
				}
			}
			break;
		case (kDriveInit) :
			printf("In DriveStraightInit\n");
			//ReadFromJetson(); //already being done in main program
			if (abort_) {
				isDone_ = true;
				nextState_ = kPivotInit;
				break;
			}

            // TODO: REDO ALLLLLLL THE MATH HERE
			if (fabs(desiredDistance_) > (2.0/12.0)) {	// ?? where did this come from lol - 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, anglePIDOutput_, distancePIDOutput_,
						robot_, desiredDistance_);
				driveStraightCommand_->Init();
				nextState_ = kDriveUpdate;
			} else {
				isDone_ = true;
				printf("Done with AlignWithTape \n");
			}
			break;

		case (kDriveUpdate) :
			if (!driveStraightCommand_->IsDone()) {
				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
				nextState_ = kDriveUpdate;
			} else {
				isDone_ = true;
				// no next state
				printf("Done with AlignWithTape \n");
			}
			break;
		default:
			printf("default in align tape\n");
			break; //unecessary?
	}
	currState_ = nextState_;
	//printf("moving to next state in align with tape\n");

	if (robot_->GetTime() - initTimeAlign_ > 4.0) {	// Timeout for align with tape command
		isDone_ = true;
		printf("align with tape is DONE TIMEOUT\n robot time is %f, init time is %f, diff time is %f\n",
			robot_->GetTime(), initTimeAlign_, robot_->GetTime() - initTimeAlign_);
	}

	*/
}

bool AlignTapeCommand::IsDone() {
    return isDone_;
}

bool AlignTapeCommand::Abort() {
	return abort_;
}

void AlignTapeCommand::Reset() {
	robot_->SetLight(false);
	isDone_ = true;
	
}

//done in main program
void AlignTapeCommand::ReadFromJetson() {
    /*
    printf("starting read from jetson\n");

	//try {
	string contents = s_recv(*subscriber_);
	printf("contents from jetson: %s\n", contents.c_str());
	stringstream ss(contents);
	vector<string> result;

	while(ss.good()) {
		string substr;
		getline( ss, substr, ' ' );
		if (substr == "") {
			continue;
		}
		result.push_back( substr );
	}
	
	contents = contents.c_str();
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
		*/
		
	/*} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cout << exc.what() << std::endl;
		desiredDeltaAngle_ = 0.0;
		// desiredDistance_ = 0.0;
	}*/
	//printf("end of read from jetson\n");
    
}

AlignTapeCommand::~AlignTapeCommand() {
	robot_->SetLight(false);
}