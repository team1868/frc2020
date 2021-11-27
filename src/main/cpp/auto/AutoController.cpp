/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/AutoController.h"

/**
 * Blank constructor
 */
AutoController::AutoController() {
	autoMode = nullptr;
}

/** 
 * Constructor, allows AutoMode to be set
 * @param autoMode an AutoMode
 */
AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

/** 
 * Sets auto mode
 * @param autoMode is an AutoMode
 */
void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

/**
 * Create a queue for automode and initializes it
 */
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

/**
 * Periodic update
 * @param currTimeSec a double that is the current time in seconds
 * @param deltaTimeSec a double that is the change in time
 */
void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	autoMode->Update(currTimeSec, deltaTimeSec);
}

/** 
 * Returns if autoMode is done
 * @return true when AutoMode is done
 */
bool AutoController::IsDone() {
	return autoMode->IsDone();
}

/** 
 * Aborts auto
 */
bool AutoController::Abort() {
	return autoMode->Abort();
}