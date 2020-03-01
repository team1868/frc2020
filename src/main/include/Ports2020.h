/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>

static const int LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT      = 6;
static const int RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT     = 8;
static const int LEFT_DRIVE_ENCODER_RED_PWM_PORT         = 7;
static const int RIGHT_DRIVE_ENCODER_RED_PWM_PORT        = 9;

// Drive Motors
static const int RIGHT_DRIVE_MASTER_ID                   = 0;
static const int RIGHT_DRIVE_SLAVE_A_ID                  = 1;
static const int LEFT_DRIVE_MASTER_ID                    = 2;
static const int LEFT_DRIVE_SLAVE_A_ID                   = 3;

// fix/set all superstructure ids
// Superstructure Motors
static const int FLYWHEEL_MOTOR_ONE_ID                   = 20; 
static const int FLYWHEEL_MOTOR_TWO_ID                   = 21;    
static const int CLIMB_RIGHT_ELEVATOR_ID                 = 26; 
static const int CLIMB_LEFT_ELEVATOR_ID                  = 25;
static const int CONTROL_PANEL_MOTOR_ID                  = 11;
static const int INTAKE_ROLLERS_MOTOR_ID                 = 10; 
static const int INTAKE_WRIST_MOTOR_ID                   = 12; 
static const int INDEX_FUNNEL_MOTOR_ID                   = 5;
static const int ELEVATOR_FEEDER_MOTOR_ID                = 4; 
static const int ELEVATOR_MOTOR_ID                       = 8;



// Pneumatics
static const int PNEUMATICS_CONTROL_MODULE_ID            = 0;

static const int GEAR_SHIFT_FORWARDS_SOLENOID_PORT       = 1; // fix
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT        = 2; // fix
static const int LIGHT_SOLENOID_PORT                     = 4; // fix, should control LED light on tape detect camera
static const int FLYWHEEL_HOOD_SOLENOID_PORT             = 0; // this is correct

// PDP Channels 
static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN             = 0; 
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN             = 1;
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN            = 15;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN            = 14;
static const int FLYWHEEL_MOTOR_ONE_PDP_CHAN             = 3; // superstructure pdp channels are all wrong
static const int FLYWHEEL_MOTOR_TWO_PDP_CHAN             = 4;
static const int CLIMB_MOTOR_ONE_PDP_CHAN                = 5;
static const int CLIMB_MOTOR_TWO_PDP_CHAN                = 6;
static const int CONTROL_PANEL_MOTOR_PDP_CHAN            = 7;
static const int INTAKE_ROLLERS_MOTOR_PDP_CHAN           = 8;
static const int INTAKE_WRIST_MOTOR_PDP_CHAN             = 9;
static const int INDEX_FUNNEL_MOTOR_PDP_CHAN             = 5;
static const int ELEVATOR_FEEDER_MOTOR_PDP_CHAN          = 11;
static const int ELEVATOR_MOTOR_PDP_CHAN                 = 12;

// Joysticks
static const int LEFT_JOY_USB_PORT                       = 0;
static const int RIGHT_JOY_USB_PORT                      = 1;
static const int OPERATOR_JOY_USB_PORT                   = 2;
static const int OPERATOR_JOY_B_USB_PORT                 = 3;

// Buttons
//static const int HIGH_GEAR_BUTTON_PORT               = 1; don't need these w/gearshift
//static const int LOW_GEAR_BUTTON_PORT                = 2;

static const int ALIGN_TAPE_BUTTON_PORT                  = 1;
static const int SHOOTING_BUTTON_PORT                    = 2;//1; // fix all superstructure button ports
static const int TRENCH_ALIGN_TAPE_BUTTON_PORT           = 5;
static const int SHOOT_CLOSE_PREP_BUTTON_PORT            = 3; //random
static const int SHOOT_FAR_PREP_BUTTON_PORT              = 2; //random
static const int UNDO_ELEVATOR_BUTTON_PORT               = 1; // fix port
static const int FUNNEL_FEEDER_ELEVATOR_UP_BUTTON_PORT   = 6;

static const int INTAKE_SERIES_BUTTON_PORT               = 1;      
static const int INDEX_SERIES_BUTTON_PORT                = 3;
static const int WRIST_DOWN_BUTTON_PORT                  = 11;
static const int WRIST_UP_BUTTON_PORT                    = 8;
static const int WRIST_RUN_ROLLERS_BUTTON                = 7;
static const int WRIST_REVERSE_ROLLERS_BUTTON            = 6;

static const int CLIMB_RIGHT_ELEVATOR_UP_BUTTON_PORT     = 4; 
static const int CLIMB_RIGHT_ELEVATOR_DOWN_BUTTON_PORT   = 5;
static const int CLIMB_LEFT_ELEVATOR_UP_BUTTON_PORT      = 4; 
static const int CLIMB_LEFT_ELEVATOR_DOWN_BUTTON_PORT    = 5;


static const int CONTROL_PANEL_BUTTON_PORT               = 51;//2;

static const int GEARSHIFT_BUTTON_PORT                   = 3; //random





// SENSORS
static const int GYRO_PORT                               = 1; // ANALOG IO

// DIO
#ifdef PRACTICE_BOT
static const int TOP_ELEVATOR_LIGHT_SENSOR_PORT          = 0;
static const int BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT       = 1;
#else 
static const int TOP_ELEVATOR_LIGHT_SENSOR_PORT          = 3;//0;
static const int BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT       = 2;//1;
#endif
static const int LEFT_CLIMB_LIMIT_SWITCH_PORT            = 4;
static const int RIGHT_CLIMB_LIMIT_SWITCH_PORT           = 5;


