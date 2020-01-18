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

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
//	printf("Auto controller update\n");
	autoMode->Update(currTimeSec, deltaTimeSec);
}

bool AutoController::IsDone() {
	return autoMode->IsDone();
}

bool AutoController::Abort() {
	return autoMode->Abort();
}