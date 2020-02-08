/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
using namespace std;

SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) :
    superstructureLayout_(robot->GetDriverTab().GetLayout("Drive Modes", "List Layout").WithPosition(0, 1))
    {
    robot_ = robot;
    humanControl_ = humanControl;

    climberPower_ = 0.5; // fix
    desiredRPM_ = 2000;
    flywheelPower_ = CalculateFlywheelPowerDesired();

    elevatorFeederPower_ = 0.3; // fix
    indexFunnelPower_ = 0.3; // fix
    flywheelResetTime_ = 5.0; // fix
    flywheelFeederPower_ = 0.3; // fix!
    pushNextBallTime_ = 1.0; // fix - minimum time it takes for elevator to move next ball right to topmost sensor

    currState_ = kInit;
	nextState_ = kIdle;

    desiredIntakeWristAngle_ = 30.0; // fix later :)

    controlPanelCounter_ = 0;

    numBalls_ = 3.0; // not real number, needs to be updated (increased) by kIndexing
	isSpeed_ = false;
    
    // create talon pid controller
    //flywheelPIDController_ = new rev::CANPIDController(*robot_->GetFlywheelMotor1());
    flywheelEncoder1_ = &robot_->GetFlywheelMotor1()->GetSensorCollection();
    flywheelEncoder2_ = &robot_->GetFlywheelMotor2()->GetSensorCollection();
    
    // shuffleboard
    flywheelVelocityEntry_ = superstructureLayout_.Add("flywheel velocity", 0.0).GetEntry();
    
    flywheelPEntry_ = superstructureLayout_.Add("flywheel P", 0.0).GetEntry();
    flywheelIEntry_ = superstructureLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = superstructureLayout_.Add("flywheel D", 0.0).GetEntry();
    flywheelFFEntry_ = superstructureLayout_.Add("flywheel FF", 0.0).GetEntry();

    funnelLightSensorEntry_ = superstructureLayout_.Add("funnel ball", false).GetEntry();
    bottomElevatorLightSensorEntry_ = superstructureLayout_.Add("bottom elevator ball", false).GetEntry();
    topElevatorLightSensorEntry_ = superstructureLayout_.Add("top elevator ball", false).GetEntry();
}

void SuperstructureController::Reset() { // might not need this
    currState_ = kInit;
	nextState_ = kIdle;
}

void SuperstructureController::Update(){
    currTime_ = robot_->GetTime();//may or may not be necessary
    RefreshShuffleboard();

    switch(currState_) {
        case kInit:
            // calibrate our gyro in autonomous init
	        //currGyroAngle_ = lastGyroAngle_ = 0.0;
	        // currTime_ = lastTime_ = 0.0;
            nextState_ = kIdle;
            break;
        case kIdle:
            cout << "idle" << endl;

            robot_->SetFlywheelOutput(0.0);
            robot_->SetClimberOutput(0.0);
            robot_->SetControlPanelOutput(0.0);
            robot_->SetClimberElevatorOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIntakeWristOutput(0.0);

            if (humanControl_->GetDesired(ControlBoard::Buttons::kHighGearShift)){
                robot_->SetHighGear();
            } else {
                robot_->SetLowGear();
            }

            //light for align tape turned on and off in align tape command
            /*if (humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
                printf("in light\n");
                robot_->SetLight(true);
            } else {
                robot_->SetLight(false);
            }*/

            /*
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
            */
            
            if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
                nextState_ = kIntaking;
            } /*else if (currState_ == kIntaking) {
                    nextState_ = kIndexing;
            } else
                nextState_ = kIdle;*/

            break;
        case kIntaking:
            if(robot_->GetGyroAngle()<desiredIntakeWristAngle_-20){
                robot_->SetIntakeWristOutput(0.3); // tune speeds
            }
            CalculateIntakeRollersPower();
            if(robot_->GetGyroAngle()<desiredIntakeWristAngle_){
                robot_->SetIntakeWristOutput(0.2);
            }
            if(!humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
                nextState_ = kIndexing;
            }
            break;
        case kIndexing:
            robot_->SetIntakeRollersOutput(0.0);
            IndexUpdate();
            //exit condition
            break;
        case kShooting:
            
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

/*
void SuperstructureController::FlywheelPIDControllerUpdate() {
    flywheelPIDController_->SetP(flywheelPFac_);
    flywheelPIDController_->SetI(flywheelIFac_);
    flywheelPIDController_->SetD(flywheelDFac_);
    flywheelPIDController_->SetFF(flywheelFFFac_);                   // renegade 
    
}*/

double SuperstructureController::CalculateFlywheelPowerDesired() {
    return 0.5; // fix
}

void SuperstructureController::CalculateIntakeRollersPower() {
    double power = abs(robot_->GetDrivePower())*2;
    if (power <= 1)
        robot_->SetIntakeRollersOutput(power);
    else
        robot_->SetIntakeRollersOutput(1.0);
}

void SuperstructureController::ControlPanelStage2(double power){
    initialControlPanelColor_ = robot_->MatchColor();
    previousControlPanelColor_ = initialControlPanelColor_;
    while (controlPanelCounter_ < 8) { //KILL THIS
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
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'G' :
                colorDesired_ = "Yellow";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'R' :
                colorDesired_ = "Blue";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                ControlPanelFinalSpin();
                break;
            case 'Y' :
                colorDesired_ = "Green";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
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
    initialControlPanelTime_ = robot_->GetTime(); // move time
    while(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time and change to if
        robot_->SetControlPanelOutput(0.3); // fix power
    }
}

bool SuperstructureController::IndexUpdate(){ //returns whether is done!
    if(robot_->GetElevatorLightSensorStatus()){
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
        robot_->SetElevatorOutput(0.0);
    } else {
        robot_->SetIndexFunnelOutput(0.5); // test power
        robot_->SetElevatorFeederOutput(0.53); // test power
        if (robot_->GetElevatorFeederLightSensorStatus()){
            robot_->SetElevatorOutput(0.5); // test power
        }
        robot_->SetElevatorOutput(0.0); //might not need this?
    }
    return true;
}

void SuperstructureController::RefreshShuffleboard(){
    //funnelLightSensorEntry_.SetBoolean(robot_->GetFunnelLightSensorStatus());
    //bottomElevatorLightSensorEntry_.SetBoolean(robot_->GetBottomElevatorLightSensorStatus());
    //topElevatorLightSensorEntry_.SetBoolean(robot_->GetTopElevatorLightSensorStatus());

    flywheelPFac_ = flywheelPEntry_.GetDouble(0.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFFac_ = flywheelFFEntry_.GetDouble(0.0);
    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
    lastGyroAngle_ = currGyroAngle_;
	currGyroAngle_ = robot_->GetGyroAngle();
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {}
