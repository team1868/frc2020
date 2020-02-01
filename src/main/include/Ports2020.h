/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>

static const int LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT  = 6;
static const int RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT = 8;
static const int LEFT_DRIVE_ENCODER_RED_PWM_PORT     = 7;
static const int RIGHT_DRIVE_ENCODER_RED_PWM_PORT    = 9;

// Drive Motors
static const int RIGHT_DRIVE_MASTER_ID               = 0;
static const int RIGHT_DRIVE_SLAVE_A_ID              = 1;
static const int LEFT_DRIVE_MASTER_ID                = 2;
static const int LEFT_DRIVE_SLAVE_A_ID               = 3;

// Superstructure Motors
static const int FLYWHEEL_MOTOR_ONE_ID               = 4; // fix/set all superstructure ids
static const int FLYWHEEL_MOTOR_TWO_ID               = 5; 
static const int CLIMB_MOTOR_ONE_ID                  = 6; 
static const int CLIMB_MOTOR_TWO_ID                  = 7; 
static const int CONTROL_PANEL_MOTOR_ID              = 8;

// Pneumatics
static const int PNEUMATICS_CONTROL_MODULE_ID        = 0;

static const int GEAR_SHIFT_FORWARDS_SOLENOID_PORT   = 0; // fix
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT    = 2; // fix
static const int LIGHT_SOLENOID_PORT                 = 4; // fix, should control LED light on tape detect camera


// PDP Channels 
static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN         = 0; 
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN         = 1;
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN        = 15;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN        = 14;
static const int FLYWHEEL_MOTOR_A_PDP_CHAN           = 3; // superstructure pdp channels are all wrong
static const int FLYWHEEL_MOTOR_B_PDP_CHAN           = 4;
static const int CLIMB_MOTOR_A_PDP_CHAN              = 5;
static const int CLIMB_MOTOR_B_PDP_CHAN              = 6;
static const int CONTROL_PANEL_PDP_CHAN              = 7;


// Joysticks
static const int LEFT_JOY_USB_PORT                   = 0;
static const int RIGHT_JOY_USB_PORT                  = 1;
static const int OPERATOR_JOY_USB_PORT               = 2;
static const int OPERATOR_JOY_B_USB_PORT             = 3;

// Buttons
static const int HIGH_GEAR_BUTTON_PORT               = 1;
static const int LOW_GEAR_BUTTON_PORT                = 2;

static const int FLYWHEEL_BUTTON_PORT                = 1; // fix all superstructure button ports
static const int CLIMBER_BUTTON_PORT                 = 2; 
static const int ALIGN_TAPE_BUTTON_PORT              = 3;
static const int CONTROL_PANEL_STAGE_2_BUTTON_PORT   = 4;
static const int CONTROL_PANEL_STAGE_3_BUTTON_PORT   = 5; 