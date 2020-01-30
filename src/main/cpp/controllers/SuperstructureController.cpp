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
            if (humanControl_ -> GetDesired(ControlBoard::Buttons::kHighGearShift)){
                robot_ -> SetHighGear();
            }
            if (humanControl_ -> GetDesired(ControlBoard::Buttons::kAlignButton)){
                cout << "in light" << endl;
                robot_ -> SetLight(true);
            } else {
                robot_ -> SetLight(false);
            }

            if(humanControl_ -> GetFlywheelDesired()){
                printf("flywheel button being pressed\n");
                cout<<"flywheel power "<<flywheelPower_<<endl;
                robot_ -> SetFlywheelOutput(flywheelPower_);
            } else {
                robot_ -> SetFlywheelOutput(0.0);
            }  

            if(humanControl_ -> GetClimberDesired()){
                printf("climber button being pressed\n");
                cout<<"climber power "<<climberPower_<<endl;
                robot_ -> SetClimberOutput(climberPower_);
            } else {
                robot_ -> SetClimberOutput(0.0);
            }  

            cout << "end update" << endl;
            break;
            
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
    currState_ = nextState_;
}


void SuperstructureController::FlywheelPIDControllerUpdate() {
    flywheelPIDController_->SetP(flywheelPFac_);
    flywheelPIDController_->SetI(flywheelIFac_);
    flywheelPIDController_->SetD(flywheelDFac_);
    flywheelPIDController_->SetFF(flywheelFFFac_);
    
}


double SuperstructureController::CalculateFlywheelPowerDesired() {
    return 0.5; // fix
}

void SuperstructureController::RefreshShuffleboard(){
    flywheelPFac_ = flywheelPEntry_.GetDouble(0.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFFac_ = flywheelFFEntry_.GetDouble(0.0);
    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
}

SuperstructureController::~SuperstructureController() {}
