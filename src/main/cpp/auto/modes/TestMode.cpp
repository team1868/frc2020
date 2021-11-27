/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/modes/TestMode.h"

/** 
 * Constructor
 * @param robot a RobotModel
 * @param controlBoard a ControlBoard 
 */
TestMode::TestMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard) {
    printf("in test mode constructor \n"); 
}

/** 
 * Overrides CreateQueue method (virtual) from AutoMode, gets queue of commands from test sequence string 
 * @param pos from AutoMode AutoPositions
 */
void TestMode::CreateQueue(AutoMode::AutoPositions pos) {
    std::string testSequence = robot_->GetTestSequence();
    // get queue of commands from test sequence string 
    QueueFromString(testSequence);
}

/** 
 * Initializes test mode
 */
void TestMode::Init() {
	printf("Initializing Test mode\n");

    // initialize current command
	currentCommand_->Init();
	printf("Finished initializing\n");
}

/** 
 * Destructor
 */
TestMode::~TestMode() {
    
}