/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "AutoMode.h"

class AlignMode : public AutoMode {
public:
    AlignMode(RobotModel *robot, ControlBoard *controlBoard);
    void CreateQueue(AutoMode::AutoPositions pos) override;
    void Init();

    virtual ~AlignMode();
};
