/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
using namespace std;

SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) :
    flywheelPIDLayout_(robot->GetSuperstructureTab().GetLayout("Flywheel", "List Layout").WithPosition(0, 1)),
    sensorsLayout_(robot->GetSuperstructureTab().GetLayout("Sensors", "List Layout").WithPosition(0, 1)),
    potLayout_(robot_->GetSuperstructureTab().GetLayout("Potentiometer", "List Layout").WithPosition(0, 1))
    {
    robot_ = robot;
    humanControl_ = humanControl;

    // fix all of this
    climbElevatorUpPower_ = 0.5; // fix
    climbElevatorDownPower_ = -0.4; // fix
    
    desiredRPM_ = 2000;
    flywheelPower_ = 0.0; //CalculateFlywheelPowerDesired();
    closeFlywheelPower_ = 0.4;
    flywheelResetTime_ = 5.0; // fix //why does this exist
    // create talon pid controller
    // fix encoder source
    flywheelEncoder1_ = &robot_->GetFlywheelMotor1()->GetSensorCollection();
    flywheelEncoder2_ = &robot_->GetFlywheelMotor2()->GetSensorCollection();
    //flywheelPID_ = new PIDController(flywheelPFac_, flywheelIFac_, flywheelEncoder1_, flywheelPIDOutput_);

    elevatorFeederPower_ = 0.3; // fix
    elevatorSlowPower_ = 0.2; //fix
    elevatorFastPower_ = 0.4; //fix
    indexFunnelPower_ = 0.3; // fix
    lowerElevatorTimeout_ = 4.0; //fix
    elevatorTimeout_ = 4.0;
    //lastBottomStatus_ = false;

    controlPanelPower_ = 0.5; // fix
    controlPanelCounter_ = 0;

    desiredIntakeWristAngle_ = 30.0; // fix later :)

    startResetTime_ = 0.0;
    resetTimeout_ = 3.0;

    currState_ = kInit;
	nextState_ = kIndexing;
    currAutoState_ = kInit;
    nextAutoState_ = kAutoIndexing;
    currIndexState_ = kIndexInit;
    nextIndexState_ = kIndexInit;
    currWristState_ = kRaising;
    

    // shuffleboard
    flywheelVelocityEntry_ = flywheelPIDLayout_.Add("flywheel velocity", 0.0).GetEntry();
    
    flywheelPEntry_ = flywheelPIDLayout_.Add("flywheel P", 0.0).GetEntry();
    flywheelIEntry_ = flywheelPIDLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = flywheelPIDLayout_.Add("flywheel D", 0.0).GetEntry();
    flywheelFFEntry_ = flywheelPIDLayout_.Add("flywheel FF", 0.0).GetEntry();

    wristPEntry_ = robot_->GetSuperstructureTab().Add("wrist P", 0.0).GetEntry();

    elevatorTopLightSensorEntry_ = sensorsLayout_.Add("bottom elevator", false).GetEntry();
    elevatorBottomLightSensorEntry_ = sensorsLayout_.Add("top elevator", false).GetEntry();
}

void SuperstructureController::Reset() { // might not need this
    currState_ = kInit;
	nextState_ = kIndexing;
    // check whether autostates should be reset here or should 
    currAutoState_ = kAutoInit;
    nextAutoState_ = kAutoIndexing;
    currWristState_ = kRaising;
}

void SuperstructureController::WristControllerUpdate(){

    currWristAngle_ = robot_ -> GetIntakeWristPotValue(); // might not need?
    switch (currWristState_){
        case kRaising:
            // might not need lowering if we have an idle
            robot_->SetIntakeRollersOutput(0.0);
            if(currWristAngle_ > 0) {
                robot_ -> SetIntakeWristOutput((0.0-currWristAngle_)*wristPFac_); // check if potentiometer returns angle
            }
            break;
        case kLowering:
            if(currWristAngle_ < desiredIntakeWristAngle_) {
                robot_ -> SetIntakeWristOutput((desiredIntakeWristAngle_-currWristAngle_)*wristPFac_);
            }
            if(currWristAngle_ > desiredIntakeWristAngle_ - 20.0){
                robot_->SetIntakeRollersOutput(CalculateIntakeRollersPower());
            }
            break;
        default:
            printf("ERROR: no state in wrist controller \n");

    }
}

void SuperstructureController::AutoUpdate(){
    RefreshShuffleboard();
    // auto states 
    switch(currAutoState_){
        case kAutoInit:
            break;
        case kAutoIndexing:
            IndexUpdate();
            break;
        case kAutoIntaking:
            IndexUpdate();
            break;
        case kAutoCloseShooting: // fix for auto, there is no next state or kresetting
            desiredFlywheelPower_ = closeFlywheelPower_;
            robot_->SetFlywheelOutput(desiredFlywheelPower_);
            /*
            if((IsFlywheelAtSpeed() || !topSensor_) && !tTimeout_){
                robot_->SetElevatorOutput(elevatorSlowPower_);
            } else {
                robot_->SetElevatorOutput(0.0);
            }

            if(!bottomSensor_ && !bTimeout_){
                robot_->SetIndexFunnelOutput(indexFunnelPower_);
                robot_->SetElevatorFeederOutput(elevatorFeederPower_);
            } else {
                robot_->SetIndexFunnelOutput(0.0);
                robot_->SetElevatorFeederOutput(0.0);
            }

            if(tTimeout_ && bTimeout_){
                nextState_ = kResetting;
                startResetTime_ = currTime_;
            }

            robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); ^ implemented above - test*/
            break;
        case kAutoFarShooting:
            desiredFlywheelPower_ = CalculateFlywheelPowerDesired();
            robot_->EngageFlywheelHood(); // hood implemented
            robot_->SetFlywheelOutput(desiredFlywheelPower_);
            /*if((IsFlywheelAtSpeed() || !topSensor_) && !tTimeout_){
                robot_->SetElevatorOutput(elevatorSlowPower_);
            } else {

                robot_->SetElevatorOutput(0.0);
            }

            if(!bottomSensor_ && !bTimeout_){
                robot_->SetIndexFunnelOutput(indexFunnelPower_);
                robot_->SetElevatorFeederOutput(elevatorFeederPower_);
            } else {
                robot_->SetIndexFunnelOutput(0.0);
                robot_->SetElevatorFeederOutput(0.0);
            }

            if(tTimeout_ && bTimeout_){
                nextState_ = kResetting;
                startResetTime_ = currTime_;
            }

            robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); ^ implemented above*/
            break;
        default:
            printf("ERROR: no state in auto update \n");
    }
}

void SuperstructureController::Update(){
    
    currTime_ = robot_->GetTime();//may or may not be necessary
    RefreshShuffleboard();

    //human override state
    if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
        currState_ = kIntaking;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton)){
        currState_ = kShooting; 
    } else if(currState_ != kResetting){
        currState_ = kIndexing;
    }
    
    //flywheel control if not shooting
    if (currState_ != kShooting){
        if(humanControl_->GetDesired(ControlBoard::Buttons::kShootClosePrepButton)){
            desiredFlywheelPower_ = closeFlywheelPower_;
            robot_->SetFlywheelOutput(desiredFlywheelPower_);
            robot_->DisengageFlywheelHood();
        } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootFarPrepButton)){
            desiredFlywheelPower_ = 0.8; //TODO REPLACE WITH VISION & CalculateFlywheelPower();
            robot_->SetFlywheelOutput(desiredFlywheelPower_);
            robot_->EngageFlywheelHood();
        } else {
            robot_->SetFlywheelOutput(0.0);
        }
    }    

    bottomSensor_ = robot_->GetElevatorFeederLightSensorStatus();
    topSensor_ = robot_->GetElevatorLightSensorStatus();
    
    if(topSensor_){
        startElevatorTime_ = currTime_;
    }
    if(bottomSensor_){
        startIndexTime_ = currTime_;
    }

    tTimeout_ = currTime_-startElevatorTime_ > elevatorTimeout_;
    bTimeout_ = currTime_-startIndexTime_ > lowerElevatorTimeout_;

    if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
        ControlPanelStage2(controlPanelPower_);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
        ControlPanelStage3(controlPanelPower_);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbElevatorUpButton)){
        robot_->SetClimberElevatorOutput(climbElevatorUpPower_);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbElevatorUpButton)){
        robot_->SetClimberElevatorOutput(climbElevatorDownPower_);
    }

    //TODO replace "//robot_->SetArm(bool a);" with if !sensorGood set arm power small in bool a direction
    //in current code: true is arm down and false is arm up
    //TODO ADD CLIMBING AND SPINNER SEMIAUTO
    switch(currState_){
        case kIndexing:
            IndexUpdate();

            robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        case kIntaking:
            robot_->SetIntakeRollersOutput(CalculateIntakeRollersPower());
            currWristState_ = kLowering;
            //robot_->SetArm(true); TODO IMPLEMENT
            IndexUpdate();
            break;
        case kShooting:
            robot_->SetFlywheelOutput(desiredFlywheelPower_);
            if((IsFlywheelAtSpeed() || !topSensor_) && !tTimeout_){
                robot_->SetElevatorOutput(elevatorSlowPower_);
            } else {
                robot_->SetElevatorOutput(0.0);
            }

            if(!bottomSensor_ && !bTimeout_){
                robot_->SetIndexFunnelOutput(indexFunnelPower_);
                robot_->SetElevatorFeederOutput(elevatorFeederPower_);
            } else {
                robot_->SetIndexFunnelOutput(0.0);
                robot_->SetElevatorFeederOutput(0.0);
            }

            if(tTimeout_ && bTimeout_){
                nextState_ = kResetting;
                startResetTime_ = currTime_;
            }

            robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        case kResetting:
            if(!bottomSensor_ && currTime_-startResetTime_ <= resetTimeout_){
                robot_->SetElevatorOutput(-elevatorFastPower_); //bring down elevator
            } else {
                robot_->SetElevatorOutput(0.0);
                nextState_ = kIndexing;
            }

            robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        default:
            printf("ERROR: no state in superstructure controller\n");
            robot_->SetFlywheelOutput(0.0);
            robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
    }

    currState_ = nextState_;

    /*
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

bool SuperstructureController::IndexUpdate(){

    printf("top sensor %f and bottom sensor %f\n", topSensor_, bottomSensor_);

    //control top
    if(!topSensor_ && bottomSensor_){
        printf("RUNNING TOP ELEVATOR\n");
        robot_->SetElevatorOutput(elevatorFastPower_);
    } else {
        robot_->SetElevatorOutput(0.0);
    }

    //control bottom
    if(!bottomSensor_ && !bTimeout_){
        robot_->SetIndexFunnelOutput(indexFunnelPower_);
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    } else {
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
    }

    if((topSensor_ && bottomSensor_) || !bTimeout_){
        return true;
    } else {
        return false;
    }
}

double SuperstructureController::CalculateFlywheelPowerDesired() {
    return 0.5; // fix
}

double SuperstructureController::CalculateIntakeRollersPower() {
    /*double power = abs(robot_->GetDrivePower())*2;
    if (power <= 1)
        robot_->SetIntakeRollersOutput(power);
    else
        robot_->SetIntakeRollersOutput(1.0);*/
    return 1.0;
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
    }
}

//TODO FIX
bool SuperstructureController::IsFlywheelAtSpeed(){
    return true;
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
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'G' :
                colorDesired_ = "Yellow";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'R' :
                colorDesired_ = "Blue";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
                initialControlPanelTime_ = robot_->GetTime();
                ControlPanelFinalSpin();
                break;
            case 'Y' :
                colorDesired_ = "Green";
                if(colorDesired_.compare(robot_->MatchColor()) != 0) {
                     robot_->SetControlPanelOutput(power);
                }
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
    nextState_ = kIndexing;
}

void SuperstructureController::ControlPanelFinalSpin() {
    if(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time and change to if
        robot_->SetControlPanelOutput(0.3); // fix power
    }
    robot_->SetControlPanelOutput(0.0);
}

void SuperstructureController::RefreshShuffleboard(){
    elevatorBottomLightSensorEntry_.SetBoolean(robot_->GetElevatorFeederLightSensorStatus());
    elevatorTopLightSensorEntry_.SetBoolean(robot_->GetElevatorLightSensorStatus());

    flywheelPFac_ = flywheelPEntry_.GetDouble(0.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFFac_ = flywheelFFEntry_.GetDouble(0.0);

    wristPFac_ = wristPEntry_.GetDouble(0.0);
    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
    //lastGyroAngle_ = currGyroAngle_;
	//currGyroAngle_ = robot_->GetGyroAngle();
    currWristAngle_ = robot_->GetIntakeWristPotValue();
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {}
