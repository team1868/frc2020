/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotModel.h"
#include "auto/AutoCommand.h"

class WaitingCommand : public AutoCommand {
public:
	
	/** 
	 * Constructor, assigns the waitTimeSec and creates the timer
   	 * @param robot a RobotModel
   	 * @param waitTimeSec a double
  	 */
	WaitingCommand(RobotModel *robot, double waitTimeSec);

	/**
	 * Initializes class for run, starts the timer
	 */
	void Init();

	/** 
	 * Checks if the timer meets the waitTimeSec. If so, isDone is set to true.
   	 * @param currTimeSec a double
   	 * @param deltaTimeSec a double
   	 */
	void Update(double currTimeSec, double deltaTimeSec);

    /**
     * Returns true if command is done
     * @return isDone_
     */
	bool IsDone();

	/**
     * Resets robot to standby 
     */
	void Reset();

	/**
	 * Destructor
	 */
	virtual ~WaitingCommand();

private:
	RobotModel * robot_;

	// time variables
	double waitTimeSec_;
	double startTime_;

	bool isDone_;
};