/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include "../RobotModel.h"
#include <vector>
#include <string>
#include <iostream>
#include <networktables/NetworkTableEntry.h>


class AutoCommand {
public:
	/**
	 * Constructor that generates a PathCommand - 
	 * If extended, allows other commands to implement these methods
	 */
	AutoCommand() {
		nextCommand_ = NULL;
	}

	/** 
     * Destructor
     */
	virtual ~AutoCommand() {}

	/** 
     * Initializes the command
     */
	virtual void Init() = 0;

	/** 
     * Periodic update
     * @param currTimeSec current time
     * @param deltaTimeSec delta time
     */
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;

    /**
     * Returns true if command is done
     * @return isDone_
     */
	virtual bool IsDone() = 0;

	/**
     * Resets robot to standby 
     */
	virtual void Reset() = 0;

	/**
     * Aborts command
     */
	virtual bool Abort() { return false; }

	/**
     * Gets the next command
	 * @return AutoCommand, the next command
     */
	AutoCommand* GetNextCommand() {
		return nextCommand_;
	}

	/**
     * Sets the next command
	 * @param nextCommand an AutoCommand
     */
	void SetNextCommand(AutoCommand* nextCommand) {
		nextCommand_ = nextCommand;
	}

private:
	AutoCommand *nextCommand_;
};