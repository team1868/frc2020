/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
using namespace std;

SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) {
    robot_ = robot;
    humanControl_ = humanControl;

    climberPower_ = 0.5; // fix
    desiredRPM_ = 2000;
    flywheelPower_ = CalculateFlywheelPowerDesired();

    currState_ = kInit;
	nextState_ = kIdle;

    desiredIntakeWristAngle_ = 90.0; // fix later :)

    controlPanelCounter_ = 0;
    
    flywheelPIDController_ = new rev::CANPIDController(*robot_->GetFlywheelMotor1());
    flywheelEncoder1_ = new rev::CANEncoder(*robot_->GetFlywheelMotor1(), rev::CANEncoder::EncoderType::kHallSensor, SPARK_ENCODER_TICKS);
    
    
    // shuffleboard
    flywheelVelocityEntry_ = frc::Shuffleboard::GetTab("Public_Display").Add("flywheel velocity", 0.0).GetEntry();
    
    flywheelPEntry_ = frc::Shuffleboard::GetTab("Public_Display").Add("flywheel P", 0.0).GetEntry();
    flywheelIEntry_ = frc::Shuffleboard::GetTab("Public_Display").Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = frc::Shuffleboard::GetTab("Public_Display").Add("flywheel D", 0.0).GetEntry();
    flywheelFFEntry_ = frc::Shuffleboard::GetTab("Public_Display").Add("flywheel FF", 0.0).GetEntry();
}

void SuperstructureController::Reset() { // might not need this
    currState_ = kInit;
	nextState_ = kIdle;
}

void SuperstructureController::Update(){
    RefreshShuffleboard();

    switch(currState_) {
        case kInit:
            break;
        case kIdle:
            cout << "idle" << endl;

            if (humanControl_->GetDesired(ControlBoard::Buttons::kHighGearShift)){
                robot_-> SetHighGear();
            }

            //light for align tape turned on and off in align tape command
            /*if (humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
                printf("in light\n");
                robot_->SetLight(true);
            } else {
                robot_->SetLight(false);
            }*/

            if(humanControl_->GetDesired(ControlBoard::Buttons::kFlywheelButton)){
                printf("flywheel button being pressed\n");
                cout<<"flywheel power "<<flywheelPower_<<endl;
                robot_->SetFlywheelOutput(flywheelPower_);
            } else {
                robot_->SetFlywheelOutput(0.0);
            }  

            if(humanControl_->GetDesired(ControlBoard::Buttons::kClimberButton)){
                printf("climber button being pressed\n");
                cout<<"climber power "<<climberPower_<<endl;
                robot_->SetClimberOutput(climberPower_);
            } else {
                robot_->SetClimberOutput(0.0);
            }  

            if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
                ControlPanelStage2(0.6); // test to see which speed works best
            } else {
                robot_->SetControlPanelOutput(0.0);
            }

            if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage3Button)){
                // need to do the logic and match for this, based on the placement of the color sensor
                ControlPanelStage3(0.5); // test to see which speed works best         
            } else {
                robot_->SetControlPanelOutput(0.0);
            }
            
            if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){

            } else {
                // bring wrist up
                robot_->SetIntakeRollersOutput(0.0);
                robot_->SetFunnelIndexOutput(0.0);
                robot_->SetTopElevatorOutput(0.0);
                robot_->SetBottomElevatorOutput(0.0);
            }

            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
    currState_ = nextState_;
}

/*
void SuperstructureController::DisabledUpdate() {
    if (humanControl_ -> GetDesired(ControlBoard::Buttons::kAlignButton)){
        cout << "in light disabled" << endl;
        robot_ -> SetLight(true);
    } else {
        robot_ -> SetLight(false);
    }
}
*/

void SuperstructureController::FlywheelPIDControllerUpdate() {
    flywheelPIDController_->SetP(flywheelPFac_);
    flywheelPIDController_->SetI(flywheelIFac_);
    flywheelPIDController_->SetD(flywheelDFac_);
    flywheelPIDController_->SetFF(flywheelFFFac_);                   // renegade 
    
}

double SuperstructureController::CalculateFlywheelPowerDesired() {
    return 0.5; // fix
}

void SuperstructureController::ControlPanelStage2(double power){
    initialControlPanelColor_ = robot_->MatchColor();
    previousControlPanelColor_ = initialControlPanelColor_;
    while (controlPanelCounter_ < 8) {
        robot_->SetControlPanelOutput(power);
        if (initialControlPanelColor_.compare(robot_->MatchColor()) == 0 && previousControlPanelColor_.compare(robot_->MatchColor()) != 0) {
            controlPanelCounter_++;
        }
        previousControlPanelColor_ = robot_->MatchColor();
    }
}

void SuperstructureController::ControlPanelStage3(double power) {
    // blue: cyan 100 (255, 0, 255)
    // green: cyan 100 yellow 100 (0, 255, 0)
    // red: magenta 100 yellow 100 (255, 0 , 0)
    // yellow: yellow 100 (255, 255, 0)


    if(robot_->GetControlPanelGameData().length() > 0)
    {
        switch(robot_->GetControlPanelGameData()[0])
        {
            case 'B' :
                colorDesired_ = "Red";
                while(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'G' :
                colorDesired_ = "Yellow";
                while(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'R' :
                colorDesired_ = "Blue";
                while(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'Y' :
                colorDesired_ = "Green";
                while(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            default :
                printf("this data is corrupt");
                break;
        }
    } else {
        printf("no data received yet");
        // no data received yet
    }
}

void SuperstructureController::ControlPanelFinalSpin() {
    initialControlPanelTime_ = robot_->GetTime();
    while(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time
        robot_->SetControlPanelOutput(0.3); // fix power
    }
}

void SuperstructureController::RefreshShuffleboard(){
    flywheelPFac_ = flywheelPEntry_.GetDouble(0.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFFac_ = flywheelFFEntry_.GetDouble(0.0);
    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
}

SuperstructureController::~SuperstructureController() {}
