/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/AutoController.h"

// Blank constructor
AutoController::AutoController() {
	autoMode = nullptr;
}

// Constructor that sets auto mode
AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

// Setting auto mode
void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

//put autoMode in queue
//Ask: Why create queue? What's the Init() for? (recursion?)
void AutoController::Init(AutoMode::AutoPositions pos) {
	printf("in autocontroller init\n");
	if (autoMode == NULL) {
		printf("autoMode is null\n");
	} else {
		autoMode->CreateQueue(pos);
		printf("done making queue\n");
		if(autoMode->IsDone()){
			printf("auto mode is done\n");
			return;
		}
		autoMode->Init();
		printf("init finished\n");
	}
}

//update time
void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	autoMode->Update(currTimeSec, deltaTimeSec);
}

//is autoMode done
bool AutoController::IsDone() {
	return autoMode->IsDone();
}

//abort auto
bool AutoController::Abort() {
	return autoMode->Abort();
}