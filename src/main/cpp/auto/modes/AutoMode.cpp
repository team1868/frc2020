/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/AutoMode.h"

AutoMode::AutoMode(RobotModel *robot, ControlBoard *controlBoard) {
    printf("constructing automode\n");

    firstCommand_ = nullptr;
	currentCommand_ = nullptr;

	robot_ = robot;
	humanControl_ = controlBoard;
	navX_ = robot_->GetNavXSource();

	talonEncoder_ = new TalonEncoderPIDSource(robot_);
	talonEncoderCurve_ = new TalonEncoderPIDSource(robot_);
	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();
	talonOutput_ = new PivotPIDTalonOutput(robot_);

	breakDesired_ = false;
	currAngle_ = 0.0;

	printf("Done constructing AutoMode\n");
}

// get queue of commands from auto sequence string 
void AutoMode::QueueFromString(std::string autoSequence) {
    firstCommand_ = nullptr;
	currentCommand_ = nullptr;
	AutoCommand *lastCommand = nullptr;
	AutoCommand* tempCommand = nullptr;

	// sets autoSequence as the contents of the stream
	iss.str (autoSequence);
	breakDesired_ = false;
	currAngle_ = 0.0;
	char command;

	if (autoSequence == "") {
		printf("NO SEQUENCE ! TRY AGAIN KID");
	}

	// get each command in autoSequence
	while ((iss >> command) && !breakDesired_) {
		printf("Command: %c, ", command);

		// get command associated with character in autoSequence
		tempCommand = GetStringCommand(command);

		// break if no more commands left
		if(tempCommand == nullptr){
			printf("ERROR: tempCommand is null in autoMode queuing");
			break;
		}

		// tempCommand is not null, so we can go ahead
		if (firstCommand_ == nullptr) { // start of command sequence
			firstCommand_ = tempCommand;
			currentCommand_ = firstCommand_;
			lastCommand = currentCommand_; // sets as lastCommand, assumes that firstCommand is the last command
		} else {
			// set last command
			lastCommand->SetNextCommand(tempCommand);
			lastCommand = lastCommand->GetNextCommand();

			// something failed, exit loop
			if(lastCommand == nullptr){
				breakDesired_ = true;
				printf("last command was NULL\n"); //this code may or may not be necessary
			}
		}
	}

	// clears the error state of the stream
	iss.clear();
}

// Given character command from autoSequence, return corresponding AutoCommand
AutoCommand* AutoMode::GetStringCommand(char command) {
	AutoCommand* tempCommand = nullptr;
	AutoCommand* commandA = nullptr;
		
	printf("current loading command is %c\n", command);

	switch(command) {
		case '[':
			char charA;
			iss >> charA;
			commandA = GetStringCommand(charA);
			tempCommand = commandA;

			charA = '\0';
			iss >> charA;
			while (charA != ']') {
				printf("in parallel %c\n", charA);
				AutoCommand* memeCommand  = GetStringCommand(charA);

				commandA->SetNextCommand(memeCommand);
				commandA = commandA->GetNextCommand();
				charA = '\0';
				iss >> charA;
			}

			double rand;
			iss >> rand;
			break;

		case 't':	// Pivots with absolute position
			double angle;
			iss >> angle;
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				currAngle_ = angle;
				printf("Angle: %f\n", angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_, talonOutput_);
			}
			break;

		case 'd':	// Drive straight
			double distance;
			bool driveSlow;
			iss >> distance;
			iss >> driveSlow;
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				printf("Distance: %f\n", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, driveSlow);
			}
			break;

		case '!': // point command
			double pointAngle;
			bool turningLeft;
			iss >> pointAngle;
			iss >> turningLeft;
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				currAngle_ = pointAngle;
				printf("point angle: %f\n", pointAngle);
				tempCommand = new PointCommand(robot_, pointAngle, true, navX_, turningLeft, talonOutput_);
			}
			break;

		case 'c':	// curve command
			double curveRadius;
			double curveAngle;
			bool turnLeft;
			bool goForward;
			iss >> curveRadius;
			iss >> curveAngle;
			iss >> turnLeft;
			iss >> goForward;
			if (IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				printf("radius: %f\n, angle: %f\n, turnleft: %d\n, goForward: %d\n", curveRadius, curveAngle, turnLeft, goForward);
				if (!turnLeft) { // turn right
					if (!goForward) {
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, false, false, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					} else{
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, false, true, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					}
				} else { // turn left
					if (!goForward) {
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, true, false, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					} else{
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, true, true, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					}
				}
			}
			break;

		case 'w': // wait command
			printf("Wait Command\n");
			double waitTime;
			iss >> waitTime;
			if (IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new WaitingCommand(robot_, waitTime);
			}
			break;

		case 's': // shooting with velocity
			printf("starting shooting\n");
			double autoVelocity;
			iss >> autoVelocity;
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new ShootingCommand(robot_, autoVelocity);
			}
			break;

		case 'b': // prepping with velocity
			printf("starting prepping\n");
			double desiredVelocity;
			iss >> desiredVelocity;
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new PreppingCommand(robot_, desiredVelocity);
			}
			break;

		case 'i': // intaking
			printf("starting intaking\n");
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new IntakingCommand(robot_);
			}
			break;

		case 'n': // indexing
			printf("starting indexing\n");
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new IndexingCommand(robot_);
			}
			break;

		case 'a': // auto align
			printf("starting auto align command\n");
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new AlignTapeCommand(robot_, navX_, talonOutput_);
			}
			break;

		case 'q': //shooting 2, without set velocity
			printf("shooting w/o set velocity");
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			}
			else { 
				tempCommand = new ShootingCommand(robot_);
			}
			break;

		case 'y': // prepping 2, without set velocity
			printf("prepping w/o set velocity");
			if(IsFailed(command)) { // fail in stream
				tempCommand = nullptr;
			} else {
				tempCommand = new PreppingCommand(robot_);
			}
			break;

		default:	// When it's not listed, don't do anything :)
			printf("Unexpected character %c detected. Terminating queue", command); // TODO trace this and make sure it won't crashh
			firstCommand_ = nullptr;
			currentCommand_ = nullptr;
			tempCommand = nullptr;
			breakDesired_ = true;
	}

	printf("Loaded a command\n");

	return tempCommand; // NULL if something went wrong
}

// TODO trace
// error in stream, returns true if something went wrong
bool AutoMode::IsFailed(char command) {
    if (iss.fail()) { // error in stream
        iss.clear(); // clear errors in stream
        printf("Unexpected character detected after %c. Terminating queue", command);
        firstCommand_ = nullptr;
        currentCommand_ = nullptr;
        breakDesired_ = true;
        return true;
    }
    return false;
}

void AutoMode::Update(double currTimeSec, double deltaTimeSec) {
    if (currentCommand_ != nullptr) {
        if (currentCommand_->IsDone()) { // if command is done, reset and move on
			printf("before reset in autmode\n");
			currentCommand_->Reset();
			printf("reset in automode\n");

			AutoCommand *nextCommand = currentCommand_->GetNextCommand();
            // delete currentCommand_;
			currentCommand_ = nextCommand;
            if (currentCommand_ != nullptr) {
                currentCommand_->Init(); // init current command
            } else {
				printf("currentCommand_ is null\n");
			}
        } else { // command is not done, continue to update
            currentCommand_->Update(currTimeSec, deltaTimeSec);
        }
    } else {
		// Current command is null, nothing to update
        printf("Done with auto mode update\n");
    }
}

// returns if done, false as long as current command is not null
bool AutoMode::IsDone() {
    return (currentCommand_ == nullptr);
}

// abort current command
bool AutoMode::Abort() {
	// always returns false??
	return currentCommand_->Abort();
}

// disable current command and goes back to the first command if not null
void AutoMode::Disable() {
    printf("Disabling\n");
    if (!IsDone()) {
        printf("Resetting current command\n");
        currentCommand_->Reset();
    }
    if (firstCommand_ != nullptr) {
        currentCommand_ = firstCommand_;
        AutoCommand* nextCommand;
        while (currentCommand_ != nullptr) {
            nextCommand = currentCommand_->GetNextCommand();
			// gets rid of memory/value that currentCommand points to, sets it to point to next command
            delete currentCommand_; //changed here
            currentCommand_ = nextCommand;
        }
    }

    printf("Successfully disabled\n");
}

AutoMode::~AutoMode(){
	// delete if they were created
	if (talonEncoder_ != nullptr) delete talonEncoder_;
	if (talonEncoderCurve_ != nullptr) delete talonEncoderCurve_;
	if (angleOutput_ != nullptr) delete angleOutput_;
	if (distanceOutput_ != nullptr) delete distanceOutput_;
}