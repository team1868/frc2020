/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Ports2020.h"
#include "ButtonReader.h"
#include <frc/WPILib.h>
#include <unordered_map>

class ControlBoard {
 public:
  enum Joysticks{ kLeftJoy, kRightJoy };
	enum Axes{ kX, kY, kZ, kLT};
  enum Buttons{kHighGearShift, kLowGearShift, kAlignButton, kFlywheelButton, kClimberButton, 
    kControlPanelStage2Button, kControlPanelStage3Button};

  ControlBoard();
  void ReadControls();
  double GetJoystickValue(Joysticks j, Axes a); 
  bool GetDesired(Buttons button);

  ~ControlBoard();

 private:
  void ReadAllButtons();
  frc::Joystick *leftJoy_, *rightJoy_, *operatorJoy_, *operatorJoyB_;
  double leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_;
  std::unordered_map<Buttons, ButtonReader*> buttons_;
};
