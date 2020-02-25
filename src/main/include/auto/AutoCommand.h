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
	 * Constructor that generates a PathCommand
	 * If extended, allows other commands to implement these methods
	 */
	AutoCommand() {
		nextCommand_ = NULL;
	}

	virtual ~AutoCommand() {}

	virtual void Init() = 0;

	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;

	virtual bool IsDone() = 0;

	virtual void Reset() = 0;

	virtual bool Abort() { return false; }

	AutoCommand* GetNextCommand() {
		return nextCommand_;
	}

	void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand_ = myNextCommand;
	}

private:
	AutoCommand *nextCommand_;
};