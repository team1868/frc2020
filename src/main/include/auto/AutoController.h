/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "modes/TestMode.h"

class AutoController {
public:
	/**
	 * Constructor AutoController that does not set AutoMode
	 */
	AutoController();

	/**
	 * Constructor AutoController that allows AutoMode to be set
	 * @param autoMode an AutoMode
	 */
	AutoController(AutoMode *autoMode);

	virtual ~AutoController() {}

	/** 
	 * Sets auto mode
	 * @param autoMode is an AutoMode
	 */
	void SetAutonomousMode(AutoMode *autoMode);

	/**
	 * Create a queue for automode and initializes it
	 */
	void Init(AutoMode::AutoPositions pos);

	/**
	 * Updates automode
	 * @param currTimeSec a double that is the current time in seconds
	 * @param deltaTimeSec a double that is the change in time
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/** 
	 * Returns if autoMode is done
	 * @return true when AutoMode is done
	 */
	bool IsDone();

	/** 
	 * Aborts auto
   	 */
	bool Abort();

private:
	AutoMode *autoMode;
};