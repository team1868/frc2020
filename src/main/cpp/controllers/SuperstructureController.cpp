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

    // fix all of this
    climberPower_ = 0.5; // fix
    desiredRPM_ = 2000;
    flywheelPower_ = CalculateFlywheelPowerDesired();

    elevatorFeederPower_ = 0.3; // fix
    elevatorSlowPower_ = 0.2; //fix
    elevatorFastPower_ = 0.4; //fix
    indexFunnelPower_ = 0.3; // fix
    flywheelResetTime_ = 5.0; // fix
    lowerElevatorTimeout_ = 6.0; //fix
    //lastBottomStatus_ = false;

    currState_ = kInit;
	nextState_ = kIdle;
    currIndexState_ = kIndexInit;
    nextIndexState_ = kIndexInit;

    desiredIntakeWristAngle_ = 30.0; // fix later :)

    controlPanelCounter_ = 0;
    
    // create talon pid controller
    //flywheelPID_ = new PIDController(flywheelPFac_, flywheelIFac_, *flywheelEncoder1_, flywheelPIDOutput_);
    // fix encoder source
    flywheelEncoder1_ = &robot_->GetFlywheelMotor1()->GetSensorCollection();
    flywheelEncoder2_ = &robot_->GetFlywheelMotor2()->GetSensorCollection();
    
    // shuffleboard
    flywheelVelocityEntry_ = superstructureLayout_.Add("flywheel velocity", 0.0).GetEntry();
    
    flywheelPEntry_ = superstructureLayout_.Add("flywheel P", 0.0).GetEntry();
    flywheelIEntry_ = superstructureLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = superstructureLayout_.Add("flywheel D", 0.0).GetEntry();
    flywheelFFEntry_ = superstructureLayout_.Add("flywheel FF", 0.0).GetEntry();

    elevatorFeederLightSensorEntry_ = superstructureLayout_.Add("feeder ball", false).GetEntry();
    elevatorLightSensorEntry_ = superstructureLayout_.Add("elevator ball", false).GetEntry();
}

void SuperstructureController::Reset() { // might not need this
    currState_ = kInit;
	nextState_ = kIdle;
}

void SuperstructureController::Update(){
    /*
    currTime_ = robot_->GetTime();//may or may not be necessary
    RefreshShuffleboard();

    switch(currState_) {
        case kInit:
            // calibrate our gyro in autonomous init
	        //currGyroAngle_ = lastGyroAngle_ = 0.0;
            currIntakeAngle_ = lastIntakeAngle_ = 0.0;
	        // currTime_ = lastTime_ = 0.0;
            nextState_ = kIdle;
            break;
        case kIdle:
            cout << "idle" << endl;

            robot_->SetFlywheelOutput(0.0);
            robot_->DisengageFlywheelHood();
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

            if (humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
                nextState_ = kControlPanelStage2;
            }

            if (humanControl_->JustPressed(ControlBoard::Buttons::kControlPanelStage3Button)){
                initialControlPanelColor_ = robot_->MatchColor(); // only press once color sensor can see the wheel
                nextState_ = kControlPanelStage3;
            }

            if (humanControl_->GetDesired(ControlBoard::Buttons::kFlywheelCloseButton)){
                nextState_ = kShooting;
            }

            if(humanControl_->GetDesired(ControlBoard::Buttons::kFlywheelFarButton)){
                nextState_ = kShooting;
            }

            //light for align tape turned on and off in align tape command
            // if (humanControl_->GetDesired(ControlBoard::Buttons::kAlignButton)){
            //     printf("in light\n");
            //     robot_->SetLight(true);
            // } else {
            //     robot_->SetLight(false);
            // }

            
            // if(humanControl_->GetDesired(ControlBoard::Buttons::kFlywheelButton)){
            //     printf("flywheel button being pressed\n");
            //     cout<<"flywheel power "<<flywheelPower_<<endl;
            //     robot_->SetFlywheelOutput(flywheelPower_);
            // } else {
            //     robot_->SetFlywheelOutput(0.0);
            // }  

            // if(humanControl_->GetDesired(ControlBoard::Buttons::kClimberButton)){
            //     printf("climber button being pressed\n");
            //     cout<<"climber power "<<climberPower_<<endl;
            //     robot_->SetClimberOutput(climberPower_);
            // } else {
            //     robot_->SetClimberOutput(0.0);
            // }  
            
            
            if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
                nextState_ = kIntaking;
            } 

            break;
        case kIntaking:
            if(robot_->GetIntakeWristPotValue()<desiredIntakeWristAngle_){
                robot_->SetIntakeWristOutput(0.3); // tune speeds
            }
            if(robot_->GetIntakeWristPotValue()>desiredIntakeWristAngle_-20){
                CalculateIntakeRollersPower();
            }
            // if(robot_->GetGyroAngle()<desiredIntakeWristAngle_){
            //     robot_->SetIntakeWristOutput(0.3); // tune speeds
            // }
            // if(robot_->GetGyroAngle()>desiredIntakeWristAngle_-20){
            //     CalculateIntakeRollersPower();
            // }
            if(!humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
                nextState_ = kIndexing;
            }
            break;
        case kIndexing:
            robot_->SetIntakeRollersOutput(0.0);
            //IndexUpdate();
            //exit condition
            break;
        case kShooting:
            if(robot_->GetDistance()>5.0){
                robot_->EngageFlywheelHood();
            }

            break;
        case kControlPanelStage2:
            ControlPanelStage2(0.2); // fix power
            break;
        case kControlPanelStage3:
            ControlPanelStage3(0.2); // fix power
            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
    currState_ = nextState_;
    */
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
    /*double power = abs(robot_->GetDrivePower())*2;
    if (power <= 1)
        robot_->SetIntakeRollersOutput(power);
    else
        robot_->SetIntakeRollersOutput(1.0);*/
}

void SuperstructureController::ControlPanelStage2(double power){
    previousControlPanelColor_ = initialControlPanelColor_;
    if (controlPanelCounter_ < 8) {
        robot_->SetControlPanelOutput(power);
        if (initialControlPanelColor_.compare(robot_->MatchColor()) == 0 && previousControlPanelColor_.compare(robot_->MatchColor()) != 0) {
            controlPanelCounter_++;
        }
        previousControlPanelColor_ = robot_->MatchColor();
    }
    if (controlPanelCounter_ >= 8) {
        robot_->SetControlPanelOutput(0.0);
        nextState_ = kIdle;
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
                robot_->SetControlPanelOutput(0.0);
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'G' :
                colorDesired_ = "Yellow";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                robot_->SetControlPanelOutput(0.0);
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'R' :
                colorDesired_ = "Blue";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                robot_->SetControlPanelOutput(0.0);
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'Y' :
                colorDesired_ = "Green";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                robot_->SetControlPanelOutput(0.0);
                initialControlPanelTime_ = robot_->GetTime();
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
    nextState_ = kIdle;
}

void SuperstructureController::ControlPanelFinalSpin() {
    if(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time and change to if
        robot_->SetControlPanelOutput(0.3); // fix power
    }
    robot_->SetControlPanelOutput(0.0);
}

bool SuperstructureController::IndexUpdate(bool readyToShoot){
    // switch(currState_){
    //     case kInit:
    //         nextState_ = kLower;
    //         startIndexTime_ = currTime_;
    //         break;
        //case kLower:
            // if (lastBottomStatus_ && !robot_->GetBottomElevatorLightSensorStatus()){
            //     startIndexTime_ = currTime_;
            // }

    // if(robot_->GetTopElevatorLightSensorStatus()){
    //     startElevatorTime_ = currTime_;
    // }
    if(robot_->GetElevatorFeederLightSensorStatus()){
        startIndexTime_ = currTime_;
        startElevatorTime_ = currTime_;
    }

    //B and !T, file upwards
    if (robot_->GetElevatorFeederLightSensorStatus() && !robot_->GetElevatorLightSensorStatus()){
        //nextState_ = kLift;
        printf("Idle elevator funnel, shift to moving elevator");
    }
    //exit condition - timeout or B
    else if (currTime_ - startIndexTime_ > lowerElevatorTimeout_ || robot_->GetElevatorFeederLightSensorStatus()){
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
        //lastLower_ = false;
        // lastBottomStatus_ = robot_->GetBottomElevatorLightSensorStatus();
        return true;
    }
    //!B
    else { 
        // if (!lastLower_){
        //     startIndexTime_ = currTime_;
        // }
        robot_->SetIndexFunnelOutput(indexFunnelPower_);
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
        // lastLower_ = true;
    }
    //break;
//case kLift:
    if (readyToShoot ||
        (!robot_->GetElevatorLightSensorStatus() &&
        robot_->GetElevatorFeederLightSensorStatus()) ||
        (!robot_->GetElevatorLightSensorStatus() &&
        (currTime_ - startElevatorTime_ < elevatorTimeout_))) {
        //!T & B || !T & !timout
        // if (!lastUpper_){
        //     startElevatorTime_ = currTime_;
        // }
        robot_->SetElevatorOutput(elevatorSlowPower_);
        // lastUpper_ = true;
    }
    else { //T || (!B & timeout)
        //nextIndexState_ = kLower;
        robot_->SetElevatorOutput(0.0);
        startIndexTime_ = currTime_; 
        // lastUpper_ = false;
    }
    // else { //!T & B || !T & !timout
    //     if (!lastUpper_){
    //         startElevatorTime_ = currTime_;
    //     }
    //     robot_->SetElevatorOutput(elevatorSlowPower_);
    //     lastUpper_ = true;
    // }
    //         break;
    // }
    // lastBottomStatus_ = robot_->GetBottomElevatorLightSensorStatus();
    return false;
}


void SuperstructureController::RefreshShuffleboard(){
    elevatorFeederLightSensorEntry_.SetBoolean(robot_->GetElevatorFeederLightSensorStatus());
    elevatorLightSensorEntry_.SetBoolean(robot_->GetElevatorLightSensorStatus());

    flywheelPFac_ = flywheelPEntry_.GetDouble(0.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFFac_ = flywheelFFEntry_.GetDouble(0.0);
    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
    //lastGyroAngle_ = currGyroAngle_;
	//currGyroAngle_ = robot_->GetGyroAngle();
    lastIntakeAngle_ = currIntakeAngle_;
    currIntakeAngle_ = robot_->GetIntakeWristPotValue();
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {}
