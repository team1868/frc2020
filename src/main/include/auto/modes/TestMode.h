/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "AutoMode.h"

class TestMode : public AutoMode {
public:
    TestMode(RobotModel *robot, ControlBoard *controlBoard);
    
    // gets queue of commands from test sequence string 
    void CreateQueue(AutoMode::AutoPositions pos) override;
    
    void Init();

    // destructor
    virtual ~TestMode();
};
