/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/BlankMode.h" 

/** 
 * Constructor
 * @param robot a RobotModel
 * @param controlBoard a ControlBoard 
 */
BlankMode::BlankMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard) {
	printf("In Blank Mode\n");
}

/** 
 * Overrides CreateQueue method (virtual) from AutoMode, gets queue of commands from test sequence string 
 * @param pos an AutoPositions
 */
void BlankMode::CreateQueue(AutoMode::AutoPositions pos) {
	printf("In Blank Mode Queue\n");
}

/** 
 * Initializes mode
 */
void BlankMode::Init() {
	printf("In Blank Mode Init\n");
}

/** 
 * Destructor
 */
BlankMode::~BlankMode() {
}