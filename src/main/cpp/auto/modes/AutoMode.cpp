/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/AutoMode.h"
//using namespace std;

AutoMode::AutoMode(RobotModel *robot, ControlBoard *controlBoard) {
    printf("constructing automode\n");

        firstCommand_ = NULL;
		currentCommand_ = NULL;
		robot_ = robot;
		humanControl_ = controlBoard;
		navX_ = robot_->GetNavXSource();
		talonEncoder_ = new TalonEncoderPIDSource(robot_);
		talonEncoderCurve_ = new TalonEncoderCurvePIDSource(robot_);
		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		breakDesired_ = false;
		currAngle_ = 0.0;

	printf("Done constructing AutoMode\n");
}

void AutoMode::QueueFromString(std::string autoSequence) {
    firstCommand_ = NULL;
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		iss.str (autoSequence);
		std::cout << std::string ("autosequence ") + autoSequence << std::endl;
		breakDesired_ = false;
		currAngle_ = 0.0;//robot_->GetNavXYaw();

		if (autoSequence == "") {
			printf("NO SEQUENCE ! TRY AGAIN KID");
		}

		//printf("Auto sequence: %s", autoSequence.c_str());
		
		AutoCommand* tempCommand = NULL;
		char command;

		while ((iss >> command) && !breakDesired_) {
			//iss >> command;
			printf("Command: %c, ", command);

			tempCommand = GetStringCommand(command);

			if(tempCommand==NULL){
				printf("ERROR: tempCommand is null in autoMode queuing");
				break;
			}

			if (firstCommand_ == NULL) {
				firstCommand_ = tempCommand;
				currentCommand_ = firstCommand_;
				lastCommand = currentCommand_;
			} else {
				lastCommand->SetNextCommand(tempCommand);
				lastCommand = lastCommand->GetNextCommand();
				if(lastCommand == NULL){
					breakDesired_ = true;
					printf("last command was NULL\n"); //this code may or may not be necessary
				}
			}
		}
		iss.clear(); //might be this?
}

//TODO ERROR MUST HAVE A COMMAND AFTER ANY SUPERSTRUCTURE COMMAND OR CRASH
AutoCommand* AutoMode::GetStringCommand(char command) {
		AutoCommand* tempCommand = NULL;
		AutoCommand* commandA = NULL;

		printf("current loading command is %c\n", command);

		switch(command) {
		case '[':
			char charA;
			iss >> charA;
			printf("Command %c ", charA);
			commandA = GetStringCommand(charA);
			tempCommand = commandA;

			charA = NULL;
			iss >> charA;
			while (charA != ']') {
				printf("in parallel %c\n", charA);
				AutoCommand* memeCommand  = GetStringCommand(charA);

				commandA->SetNextCommand(memeCommand);
				commandA = commandA->GetNextCommand();
				charA = NULL;
				iss >> charA;
			}

			double rand;
			iss >> rand;
			break;
		case 't':	// Pivots with absolute position
			double angle;
			iss >> angle;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				currAngle_ = angle;
				printf("Angle: %f\n", angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_);
			}
			break;
		case 'd':	// Drive straight
			double distance;
			bool slow;
			iss >> distance;
			iss >> slow;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				printf("Distance: %f\n", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, slow);
			}
			break;
		case '!':
			double pointAngle;
			bool turningLeft;
			iss >> pointAngle;
			iss >> turningLeft;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				currAngle_ = pointAngle;
				printf("point angle: %f\n", pointAngle);
				tempCommand = new PointCommand(robot_, pointAngle, true, navX_, turningLeft);
			}
			break;
		case 'c':	// curve command
			double curveRadius;
			double curveAngle;
			int turnLeft;
			int goForward;
			iss >> curveRadius;
			iss >> curveAngle;
			iss >> turnLeft;
			iss >> goForward;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				printf("radius: %f\n, angle: %f\n, turnleft: %d\n, goForward: %d\n", curveRadius, curveAngle, turnLeft, goForward);
				if (turnLeft == 0) { //todo change these to bools
					if (goForward == 0) {
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, false, false, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					} else{
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, false, true, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					}
				} else {
					if (goForward == 0) {
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, true, false, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					} else{
						tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, true, true, navX_, talonEncoderCurve_, angleOutput_, distanceOutput_);
					}
				}
			}
			break;
		case 'w':
			printf("Wait Command\n");
			double waitTime;
			iss >> waitTime;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new WaitingCommand(robot_, waitTime);
			}
			break;
		case 's': //shooting
			printf("starting shooting\n");
			double autoVelocity;
			iss >> autoVelocity;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new ShootingCommand(robot_, autoVelocity);
			}
			break;
		case 'b': //prepping
			printf("starting prepping\n");
			double desiredVelocity;
			iss >> desiredVelocity;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new PreppingCommand(robot_, desiredVelocity);
			}
			break;
		case 'i': //intaking
			printf("starting intaking\n");
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new IntakingCommand(robot_);
			}
			break;
		case 'n': //indexing
			printf("starting indexing\n");
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new IndexingCommand(robot_);
				std::cout << "making new indexing command" << std::endl;
			}
			break;
		default:	// When it's not listed, don't do anything :)
			printf("Unexpected character %c detected. Terminating queue", command);
			firstCommand_ = NULL;
			currentCommand_ = NULL;
			tempCommand = NULL;
			breakDesired_ = true;
			//break;
		}

		printf("Loaded a command\n");

		return tempCommand;
}

bool AutoMode::IsFailed(char command) {
    if (iss.fail()) {
        iss.clear();
        printf("Unexpected character detected after %c. Terminating queue", command);
        firstCommand_ = NULL;
        currentCommand_ = NULL;
        breakDesired_ = true;
        return true;
    }
    return false;
}

void AutoMode::Update(double currTimeSec, double deltaTimeSec) {
	//printf("Currently Updating \n");
    if (currentCommand_ != NULL) {
        //			printf("Update in automode running\n");
		// if (currentCommand_->Abort()) {
		// 	currentCommand_->Reset();
		// 	printf("aborting auto sequence. start driver control\n");
		// }
        if (currentCommand_->IsDone()) {
            //				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
			printf("before reset in autmode\n");
			currentCommand_->Reset();
			printf("reset in automode\n");
			AutoCommand *nextCommand = currentCommand_->GetNextCommand();
			// currentCommand_->~AutoCommand();
            // delete currentCommand_;
			currentCommand_ = nextCommand;
            if (currentCommand_ != NULL) {
                //					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
                currentCommand_->Init();
                printf("Initializing current commmand\n");
				// IndexingCommand *tempthing = dynamic_cast<IndexingCommand*>(currentCommand_);
				// if(tempthing==NULL){
				// 	printf("THIS IS NOT A INDEXING COMMAND\n");
				// }
				printf("AM I DONE???? %d\n", currentCommand_->IsDone());
            } else {
				printf("currentCommand_ is null\n");
			}
        } else {
            //				printf("Update current command\n");
            currentCommand_->Update(currTimeSec, deltaTimeSec);
        }
    } else {
        //			printf("Done with auto mode update\n");
    }
}

bool AutoMode::IsDone() {
    return (currentCommand_ == NULL);
}

bool AutoMode::Abort() {
	return currentCommand_->Abort();
}

void AutoMode::Disable() {
    printf("Disabling\n");
    if (!IsDone()) {
        printf("Resetting current command\n");
        currentCommand_->Reset();
    }
    if (firstCommand_ != NULL) {
        currentCommand_ = firstCommand_;
        AutoCommand* nextCommand;
        while (currentCommand_ != NULL) {
            nextCommand = currentCommand_->GetNextCommand();
            delete currentCommand_; //changed here
            currentCommand_ = nextCommand;
        }
    }

    printf("Successfully disabled\n");
}

AutoMode::~AutoMode(){
	if(talonEncoder_ != NULL) delete talonEncoder_;
	if(talonEncoderCurve_ != NULL) delete talonEncoderCurve_;
	if(angleOutput_ != NULL) delete angleOutput_;
	if(distanceOutput_ != NULL) delete distanceOutput_;
}