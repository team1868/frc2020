/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. Tb      code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
#include <math.h>

SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) :
    flywheelPIDLayout_(robot->GetSuperstructureTab().GetLayout("Flywheel", "List Layout").WithPosition(0, 0)),
    sensorsLayout_(robot->GetSuperstructureTab().GetLayout("Sensors", "List Layout").WithPosition(0, 1)),
    manualOverrideLayout_(robot->GetModeTab().GetLayout("climb override", "List Layout").WithPosition(1,1)),
    powerLayout_(robot->GetSuperstructureTab().GetLayout("power control", "List Layout").WithPosition(3, 0))
    {
    
    robot_ = robot;
    humanControl_ = humanControl; 

    // fix all of this
    climbElevatorUpPower_ = 0.5; // fix
    climbElevatorDownPower_ = -0.4; // fix
    climbPowerDesired_ = 0.0; // needs to equal 0
    
    desiredFlywheelVelocity_ = 0.0;//7000; // ticks per 0.1 seconds
    closeFlywheelVelocity_ = 3550;//0.55;//5000.0*2048.0/60000.0;//2000; //5000 r/m * 2048 tick/r / (60 s/m * 1000 ms/s)
    flywheelResetTime_ = 2.0; // fix //why does this exist
    stopDetectionTime_ = 0.0;

    elevatorFeederPower_ = 1.0; // fix
    elevatorSlowPower_ = 0.4; //fix
    elevatorFastPower_ = 0.4;//0.75; //fix
    indexFunnelPower_ = 0.3; // fix
    intakeRollersPower_ = 0.5;
    lowerElevatorTimeout_ = 2.0; //fix
    elevatorTimeout_ = 2.0;
    //lastBottomStatus_ = false;
    manualRollerPower_ = 0.5;
    autoWristDownP_ = 0.07;
    autoWristUpP_ = 0.1;

    controlPanelPower_ = 0.5; // fix
    controlPanelCounter_ = 0;
    controlPanelStage2_ = false;
    controlPanelStage3_ = false;

    desiredIntakeWristAngle_ = 150.0;//237.0; //down

    closePrepping_ = false;
    farPrepping_ = false;
    atTargetSpeed_ = false;
    numTimeAtSpeed_ = 0.0;

    currTime_ = robot_->GetTime();
    shootPrepStartTime_ = currTime_;
    startResetTime_ = currTime_;
    resetTimeout_ = 2.0;

    currState_ = kDefaultTeleop;
    nextState_ = kDefaultTeleop;
    currWristState_ = kRaising;
    nextWristState_ = kRaising;
    currHandlingState_ = kIndexing;
    nextHandlingState_ = kIndexing;
    
    currTime_ = robot_->GetTime();
    startElevatorTime_ = currTime_;
    startIndexTime_ = currTime_-lowerElevatorTimeout_-1.0;
    startResetTime_ = currTime_-elevatorTimeout_-1.0;

    // shuffleboard
    flywheelVelocityEntry_ = flywheelPIDLayout_.Add("flywheel velocity", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelVelocityErrorEntry_ = flywheelPIDLayout_.Add("flywheel error", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    
    flywheelPEntry_ = flywheelPIDLayout_.Add("flywheel P", 11.0).GetEntry();
    flywheelIEntry_ = flywheelPIDLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = flywheelPIDLayout_.Add("flywheel D", 0.0).GetEntry();
    //flywheelFEntry_ = flywheelPIDLayout_.Add("flywheel FF", 1.0).GetEntry();
    targetSpeedEntry_ = flywheelPIDLayout_.Add("target speed", atTargetSpeed_).GetEntry();
    flywheelMotor1OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 output", robot_->FlywheelMotor1Output()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor2OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 output", robot_->FlywheelMotor2Output()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor1CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 current", robot_->GetFlywheelMotor1Current()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor2CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 current", robot_->GetFlywheelMotor2Current()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();


    autoWristEntry_ = manualOverrideLayout_.Add("auto wrist", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    autoWristDownPEntry_ = robot_->GetPIDTab().Add("wrist down p", autoWristDownP_).GetEntry();
    autoWristUpPEntry_ = robot_->GetPIDTab().Add("wrist up p", autoWristUpP_).GetEntry();
    intakeWristAngleEntry_ = sensorsLayout_.Add("intake wrist angle", 0.0).GetEntry();

    slowElevatorEntry_ = powerLayout_.Add("slow elevator", elevatorSlowPower_).GetEntry();
    fastElevatorEntry_ = powerLayout_.Add("fast elevator", elevatorFastPower_).GetEntry();
    funnelEntry_ = powerLayout_.Add("funnel", indexFunnelPower_).GetEntry();
    rollerManualEntry_ = powerLayout_.Add("manual rollers", manualRollerPower_).GetEntry();
    closeFlywheelEntry_ = powerLayout_.Add("close flywheel", closeFlywheelVelocity_).GetEntry();

    elevatorTopLightSensorEntry_ = sensorsLayout_.Add("top elevator", false).GetEntry();
    elevatorBottomLightSensorEntry_ = sensorsLayout_.Add("bottom elevator", false).GetEntry();

    controlPanelColorEntry_ = robot_->GetSuperstructureTab().Add("control panel color", "").GetEntry();
    controlPanelColorEntry_.SetString(GetControlPanelColor());
    //TODO make timeout

    printf("end of superstructure controller constructor\n");

    shootingIsDone_ = false;
}

void SuperstructureController::Reset() { // might not need this
    currState_ = kDefaultTeleop;
    nextState_ = kDefaultTeleop;
    currHandlingState_ = kIndexing;
    nextHandlingState_ = kIndexing;
    currWristState_ = kRaising;
    nextWristState_ = kRaising;

    flywheelPFac_ = flywheelPEntry_.GetDouble(11.0);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    //flywheelFFac_ = flywheelFEntry_.GetDouble(0.0);
    FlywheelPIDControllerUpdate();
}

void SuperstructureController::WristUpdate(){
    //printf("wrist update\n");
    if(!autoWristEntry_.GetBoolean(true)){
        //printf("HERE OISDHAFOSDKJFLASKDHFJ\n");
        // human: decide rollers auto or manual
        if (humanControl_->GetDesired(ControlBoard::Buttons::kWristUpButton)){
            robot_->SetIntakeWristOutput(-0.5);
            //printf("wrist up\n");
        }
        else if (humanControl_->GetDesired(ControlBoard::Buttons::kWristDownButton)){
            robot_->SetIntakeWristOutput(0.5);
            //printf("wrist down\n");
        }
        else{
            robot_->SetIntakeWristOutput(0.0);
        }
        if(humanControl_->GetDesired(ControlBoard::Buttons::kReverseRollersButton)){
            robot_->SetIntakeRollersOutput(-0.5);
        } else if(humanControl_->GetDesired(ControlBoard::Buttons::kRunRollersButton)){
            //printf("RUNNING ROLLERS RIGH NOWWWWW at %f\n", manualRollerPower_);
            robot_->SetIntakeRollersOutput(manualRollerPower_);
        }else {
            robot_->SetIntakeRollersOutput(0.0);
        }
    } else {
        //("inside auto wrist in mode %d\n", currWristState_);
        currWristAngle_ = robot_->GetIntakeWristAngle(); // might not need?
        switch (currWristState_){
            case kRaising:
                robot_->SetIntakeRollersOutput(0.0);
                //printf("current wrist angle %f\n", currWristAngle_);
                if(currWristAngle_ > 10.0) {
                    robot_->SetIntakeWristOutput(autoWristUpP_*(0.0-currWristAngle_));//(0.0-currWristAngle_)*wristPFac_); 
                    //printf("DONE LOLS\n");
                    //robot_->SetIntakeWristOutput(-0.5);
                }
                else{
                    robot_->SetIntakeWristOutput(0.0);
                }
                break;
            case kLowering:
                //printf("lowering, pfac: %f, desired angle: %f, current angle %f\n", wristPFac_, desiredIntakeWristAngle_, currWristAngle_);
                if(currWristAngle_ < desiredIntakeWristAngle_-45.0) {
                    robot_->SetIntakeWristOutput(autoWristDownP_*(desiredIntakeWristAngle_-currWristAngle_));//(desiredIntakeWristAngle_-currWristAngle_)*wristPFac_);
                    //robot_->SetIntakeWristOutput(0.5);
                } else{
                    robot_->SetIntakeWristOutput(0.0);
                    //printf("LOWERED, not running wrist\n");
                }
                if(currWristAngle_ > desiredIntakeWristAngle_ - 45.0 - 45.0){ //within acceptable range, ~740 degrees in sensor is 90 degrees on wrist
                    robot_->SetIntakeRollersOutput(intakeRollersPower_);
                }
                else{
                    robot_->SetIntakeRollersOutput(0.0);
                }
                break;
            default:
                printf("ERROR: no state in wrist controller \n");
                robot_->SetIntakeWristOutput(0.0);
                robot_->SetIntakeRollersOutput(0.0);
        }
        currWristState_ = nextWristState_;

    }
}

void SuperstructureController::UpdatePrep(bool isAuto){
    if (!isAuto){
        UpdateButtons(); //moved button/state code into that function B)
    }  else if(!farPrepping_ && !closePrepping_ && currHandlingState_ != kShooting){ //farPrepping_ biconditional kPrepping :(((((
        desiredFlywheelVelocity_ = 0.0;
        SetFlywheelPowerDesired(0.0);//desiredFlywheelVelocity_);
        robot_->SetFlywheelOutput(0.0);
        robot_->DisengageFlywheelHood(); //TODO add if distance > x
    }
        //SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION

        //MOVED FLYWHEEL VELOCITY SETTING TO SetPreppingState()
    
}
// START OF NEW STATE MACHINE!! - DO NOT TOUCH PLS
void SuperstructureController::Update(bool isAuto){
    //std::cout << "CURRENT STATE: " << currHandlingState_ << std::endl;
    //printf("UPDATING \n");
    currTime_ = robot_->GetTime(); // may or may not be necessary
    RefreshShuffleboard();
    if (!isAuto){
        CheckClimbDesired();
        CheckControlPanelDesired();
    }
    

    switch(currState_){
        case kDefaultTeleop:
            
            
            // should these be inside or outside power cell handling
            //CheckControlPanelDesired(); // might have to move out of the DefaultTeleop
            //CheckClimbDesired();

            UpdatePrep(isAuto);
            WristUpdate();
            //printf("default teleop\n");
            // if (!isAuto){
            //     UpdateButtons();   
            // }
            //UpdateButtons();   

            IndexPrep();

            //TODO replace "//robot_->SetArm(bool a);" with if !sensorGood set arm power small in bool a direction
            //in current code: true is arm down and false is arm up
            //TODO ADD CLIMBING AND SPINNER SEMIAUTO
            switch(currHandlingState_){
                case kIntaking:
                    //printf("intaking state\n");
                    Intaking();
                    break;
                case kIndexing:
                    Indexing();
                    break;
                case kShooting:
                    //std::cout << "we in kShooting B)" << std::endl;
                    shootingIsDone_ = Shooting(isAuto_);
                    break;
                case kResetting:
                    Resetting();
                    break;
                case kUndoElevator:
                    UndoElevator();
                    break;
                default:
                    printf("ERROR: no state in Power Cell Handling \n");
            }
            //printf("DESIRED FLYWHEEL VELOCITY %f\n", desiredFlywheelVelocity_);
            currHandlingState_ = nextHandlingState_;
            
            break;
        case kControlPanel:
            printf("control panel state \n");
            robot_->SetControlPanelOutput(controlPanelPower_);
            if(!humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
                robot_->SetControlPanelOutput(0.0);
                nextState_ = kDefaultTeleop;
            }
            break;
        case kClimbing:
            printf("climbing state \n");
            switch(currClimbingState_){
                case kClimbingIdle:
                    robot_->SetRightClimberElevatorOutput(0.0);
                    robot_->SetLeftClimberElevatorOutput(0.0);
                    printf("climbing mechanisms are idle \n");
                    break;
                case kClimbingElevator:
                    robot_->SetRightClimberElevatorOutput(climbPowerDesired_);
                    robot_->SetLeftClimberElevatorOutput(climbPowerDesired_);
                    if(!humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton) &&
                    !humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton) &&
                    !humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton) &&
                    !humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)){
                        currClimbingState_ = kClimbingIdle;
                        nextState_ = kDefaultTeleop; // verify that it should be next state
                    }
                    break;
                default:
                    printf("ERROR: no state in climbing \n");
            }
            break;
        default:
            printf("ERROR: no state in superstructure controller\n");
            desiredFlywheelVelocity_ = 0.0;
            SetFlywheelPowerDesired(0.0);
            robot_->SetFlywheelOutput(0.0);
            //robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            nextWristState_ = kRaising;

    }
    currState_ = nextState_;
}

void SuperstructureController::UpdateButtons(){
    if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
        nextHandlingState_ = kIntaking;
        //printf("STARTED INTAKING YAY\n");
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton)/* && 
            currTime_ - shootPrepStartTime_ > 1.0*/){ //TODO remove this
        if(nextHandlingState_!=kShooting){
            startIndexTime_ = currTime_;
        }
        nextHandlingState_ = kShooting; 
    } else if(/*!humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton) && */
            nextHandlingState_ == kShooting){ //shooting to decide not to shoot
        nextHandlingState_ = kResetting;
    } else if(nextHandlingState_ != kResetting){ //not intaking, shooting, or resetting, only option is indexing or prepping (also includes indexing)
        nextHandlingState_ = kIndexing;
    }

    PowerCellHandlingState previousState = nextHandlingState_; //TODO ERROR bad naming, keep the same type for same name
    //printf("-----saved last handling state!-----\n");
    if(humanControl_->GetDesired(ControlBoard::Buttons::kUndoElevatorButton)){
        printf("elevator is being undone\n");
        nextHandlingState_ = kUndoElevator;
    } else if(nextHandlingState_ == kUndoElevator && !humanControl_->GetDesired(ControlBoard::Buttons::kUndoElevatorButton)) {
        nextHandlingState_ = previousState;
    }

    //flywheel control if not shooting
    if (nextHandlingState_ != kShooting){
        if(humanControl_->GetDesired(ControlBoard::Buttons::kShootClosePrepButton)){
            if(!closePrepping_){
                shootPrepStartTime_ = currTime_;
                printf("start close prep shooting\n");
            }
            printf("in close PREPPING -------------\n");
            desiredFlywheelVelocity_ = closeFlywheelVelocity_;
            //robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->DisengageFlywheelHood();
            closePrepping_ = true;
            farPrepping_ = false;
        } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootFarPrepButton)){
            printf("in far PREPPING -------------\n");
            if(!farPrepping_){
                printf("start far prep shooting\n");
                shootPrepStartTime_ = currTime_;
            }
            desiredFlywheelVelocity_ = CalculateFlywheelVelocityDesired();
            //robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
            SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION
            robot_->EngageFlywheelHood(); //TODO add if distance > x
            closePrepping_ = false;
            farPrepping_ = true;
        } else {
            //printf("STOPPING FLYWHEEL\n");
            desiredFlywheelVelocity_ = 0.0;
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->SetFlywheelOutput(0.0);
            robot_->DisengageFlywheelHood();
        }
    } 

}

void SuperstructureController::CheckControlPanelDesired(){
    SuperstructureState previousState = currState_;
    if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
        controlPanelStage2_ = true;
        controlPanelStage3_ = false;
        currState_ = kControlPanel; // would this be currState_ or nextState_
    } /*else if(!humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
        currState_ = previousState;
        robot_->SetControlPanelOutput(0.0);
    }*/
}

void SuperstructureController::CheckClimbDesired(){
    SuperstructureState previousState = currState_;
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton)){
        climbPowerDesired_ = climbElevatorDownPower_;
        currClimbingState_ = kClimbingElevator;
        currState_ = kClimbing;
    } else if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton)){
        climbPowerDesired_ = climbElevatorUpPower_;
        currClimbingState_ = kClimbingElevator;
        currState_ = kClimbing;
    } else{
        robot_->SetRightClimberElevatorOutput(0.0);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton)){
        climbPowerDesired_ = climbElevatorDownPower_;
        currClimbingState_ = kClimbingElevator;
        currState_ = kClimbing;
    } else if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)){
        climbPowerDesired_ = climbElevatorUpPower_;
        currClimbingState_ = kClimbingElevator;
        currState_ = kClimbing;
    } else{
        robot_->SetLeftClimberElevatorOutput(0.0);
    }

    // might need to move to the state machine
    if(!humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton) &&
    !humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton)){
        robot_->SetRightClimberElevatorOutput(0.0);
    }
    if(!humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton) &&
    !humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)){
        robot_->SetLeftClimberElevatorOutput(0.0);
    }
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

void SuperstructureController::IndexPrep(){
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
}

void SuperstructureController::Intaking(){
    printf("kIntaking\n");
    //robot_->SetIntakeRollersOutput(CalculateIntakeRollersPower());
    nextWristState_ = kLowering;
    if(!farPrepping_ && !closePrepping_){
        SetFlywheelPowerDesired(0.0);
        robot_->SetFlywheelOutput(0.0);
    }
    
    IndexUpdate();
}

void SuperstructureController::Indexing(){
    //std::cout << "kIndexing B)))))))))" << std::endl << std::flush;
    IndexUpdate();
    //printf("in kIndexing\n");
    if(!farPrepping_ && !closePrepping_){
        SetFlywheelPowerDesired(0.0);
        robot_->SetFlywheelOutput(0.0);
    }
    //robot_->SetIntakeRollersOutput(0.0);
    nextWristState_ = kRaising;
}

bool SuperstructureController::Shooting(bool isAuto) {
    std::cout << "kShooting" << std::endl;
    // if(farShooting_){
    //     robot_->EngageFlywheelHood();
    // }
    // else {
    //     robot_->DisengageFlywheelHood();
    // }
    //printf("in kShooting AAAAAAAAAAAAAAAA NOTICE ME !!! !! AAAA\n");// with %f\n", desiredFlywheelVelocity_);
    //robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    //std::cout << "velocity " << robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM << std::endl;
    //raise elevator if not at speed, OR nothing at top and not timed out at bottom
    if(robot_->IsAutoFlywheelAtSpeed(desiredFlywheelVelocity_) || (!topSensor_ && !bTimeout_)){
        robot_->SetElevatorOutput(elevatorSlowPower_);
    } else {                                                                                                
        robot_->SetElevatorOutput(0.0);
    }

    if(!bottomSensor_ && !bTimeout_){
        //robot_->SetIndexFunnelOutput(indexFunnelPower_); //TODO PUT BACK IN
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    } else {
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
    }

    nextWristState_ = kRaising;

    //we have stopDetectionTime_ < 0.001 because we don't want to keep setting it to currTime_ (if we do that we'll never stop shooting)
    
    if(tTimeout_ && bTimeout_/* && stopDetectionTime_ < 0.001*/){ //stopping shooting i guess D:
        // stopDetectionTime_ = robot_->GetTime();
        // std::cout << "stop detecting " << stopDetectionTime_ << std::endl;
        return true;
    }
    return false;

    // if(robot_->GetTime() >= stopDetectionTime_ + 1.0 && stopDetectionTime_ > 0.001){
    //     return true;
    // }
    // return false;
    //     }
    // return false;

    //robot_->SetIntakeRollersOutput(0.0);
    
    //robot_->SetArm(false); 
}

void SuperstructureController::Resetting() {
    printf("in kResetting\n");
    desiredFlywheelVelocity_ = 0.0;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    robot_->SetFlywheelOutput(0.0);

    if(!bottomSensor_ && currTime_-startResetTime_ <= resetTimeout_){
        robot_->SetElevatorOutput(-elevatorFastPower_); //bring down elevator
    } else {
        robot_->SetElevatorOutput(0.0);
        nextHandlingState_ = kIndexing;
    }

    //robot_->SetIntakeRollersOutput(0.0);
    robot_->SetIndexFunnelOutput(0.0);
    robot_->SetElevatorFeederOutput(0.0);
    nextWristState_ = kRaising;
    //robot_->SetArm(false); TODO IMPLEMENT
}

void SuperstructureController::UndoElevator(){
    robot_->SetElevatorOutput(-elevatorSlowPower_);
    robot_->SetElevatorFeederOutput(-elevatorFeederPower_);
    robot_->SetIndexFunnelOutput(-indexFunnelPower_);
}
void SuperstructureController::IndexUpdate(){
    //std::cout << "in indexupdate" << std::endl << std::flush;
    //printf("i n t a k i n g ?");

    //printf("top sensor %f and bottom sensor %f\n", topSensor_, bottomSensor_);

    //control top
    if(!topSensor_ && bottomSensor_){
        //printf("RUNNING TOP ELEVATOR\n");
        //printf("running elevator");
        robot_->SetElevatorOutput(elevatorFastPower_);
        std::cout << "making elevator go" << std::endl << std::flush;
    } else {
        //printf("not running elevator");
        robot_->SetElevatorOutput(0.0);
    }

    //control bottom
    if(!bottomSensor_ && (!bTimeout_ || currHandlingState_ == kIntaking)){
        robot_->SetIndexFunnelOutput(indexFunnelPower_); //TODO PUT BACK IN
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
        //printf("RUNNNNINGGGG FUNNEL AND FEEDER\n");
        //std::cout << "intake stuffs, if in kindexing B)" <<std::endl << std::flush;
    } else {
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
        //std::cout << "stopping indexing" << std::endl << std::flush;          
    }

    // if((topSensor_ && bottomSensor_) || !bTimeout_){
    //     return true;
    // } else {
    //     return false;
    // }
}

bool SuperstructureController::GetShootingIsDone(){
    return shootingIsDone_;
}

// bool SuperstructureController::GetWaitingIsDone(){
//     if(topSensor_){
//         std::cout << "ball in top of elevator" << std::endl;
//         return true;
//     }
//     return false;
// }

void SuperstructureController::SetShootingState(double autoVelocity){
    //robot_->SetLight(true);
    //distanceToTarget_ = robot_->GetDistance();
    //desiredFlywheelVelocity_ = (distanceToTarget_+1827.19)/0.547; //velocity from distance, using desmos
    desiredFlywheelVelocity_=autoVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    nextWristState_ = kRaising; //resetting whatever intake did
    nextHandlingState_ = kShooting;
    robot_->EngageFlywheelHood();

    printf("start Shooting\n");
}
void SuperstructureController::SetIndexingState(){
    std::cout << "setting index state" << std::endl  << std::flush;
    nextWristState_ = kRaising; //resetting whatever intake did
    nextHandlingState_ = kIndexing;
    printf("start Indexing-----------I AM HERE ALKDJFLAKSDJFLSAKDJFLAKSJDF-\n");
}
void SuperstructureController::SetIntakingState(){
    printf("WE ARE IN THE INTAKING STATE IN SPC.CPP THANK U :D\n");
    nextWristState_ = kLowering;
    nextHandlingState_ = kIntaking;
    printf("start Intaking\n");
}

void SuperstructureController::SetPreppingState(double desiredVelocity){ //starts warming up shooter B)
    std::cout << "prepping SDJFSKLDFJ)(WEFJLKJFLSKDJF)(SEFJSKLFJSDLKFJ" << std::endl;
    //robot_->SetLight(true);
    //distanceToTarget_ = robot_->GetDistance();
    //desiredVelocity = (distanceToTarget_+1827.19)/0.547; //velocity from distance, using desmos
    //std::cout <<  "velocity " << robot_->GetFlywheelMotor1Velocity() << std::endl;
    nextWristState_ = kRaising; //resetting whatever intake did
    if(!farPrepping_){ 
        shootPrepStartTime_ = robot_->GetTime(); //TODO FIX
        printf("start Prepping\n");
    }
    robot_->EngageFlywheelHood();
    std::cout << "flywheel hood ENGAGED" << std::endl;
    farPrepping_ = true;
    closePrepping_ = false;
    desiredFlywheelVelocity_ = desiredVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    std::cout << "supposedly set flywheelpowerdesired to " << desiredFlywheelVelocity_ <<std::endl;
}



void SuperstructureController::FlywheelPIDControllerUpdate() {

    flywheelFFac_ = RatioFlywheel();
    //printf("flywheel fFac %f\n", flywheelFFac_);
    robot_->ConfigFlywheelP(flywheelPFac_);
    robot_->ConfigFlywheelI(flywheelIFac_);
    robot_->ConfigFlywheelD(flywheelDFac_);

    /*double adjustedVelocity = desiredFlywheelVelocity_/FALCON_TO_RPM;
    double maxVelocity = RatioFlywheel()/FALCON_TO_RPM;
    double adjustedValue = (adjustedVelocity/maxVelocity*1023/adjustedVelocity);
    flywheelFFac_ = adjustedValue;
    robot_->ConfigFlywheelF(flywheelFFac_);*/
    // closed-loop error maybe?

}

double SuperstructureController::RatioFlywheel(){
    return MAX_FALCON_RPM*robot_->GetVoltage()/RATIO_BATTERY_VOLTAGE;
}

double SuperstructureController::CalculateFlywheelVelocityDesired() {
    double desiredVelocity = 1.82827*robot_->GetDistance() + 3340.61;
    //return desiredVelocity;
    return closeFlywheelVelocity_;
}

//TODO actually implement
void SuperstructureController::SetFlywheelPowerDesired(double flywheelVelocityRPM) {
    double adjustedVelocity = flywheelVelocityRPM/FALCON_TO_RPM;
    double maxVelocity = RatioFlywheel()/FALCON_TO_RPM;
    double adjustedValue = (adjustedVelocity/maxVelocity*1023/adjustedVelocity);
    flywheelFFac_ = adjustedValue;
    robot_->ConfigFlywheelF(flywheelFFac_);
    
    robot_->SetControlModeVelocity(flywheelVelocityRPM/FALCON_TO_RPM);
}

bool SuperstructureController::IsFlywheelAtSpeed(){
    if(robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM > desiredFlywheelVelocity_ && 
        robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM < desiredFlywheelVelocity_+150.0){
        numTimeAtSpeed_++;
        if (numTimeAtSpeed_ >= 1){
            atTargetSpeed_ = true;
            return true;
        }
        else{
            atTargetSpeed_ = false;
            return false;
        }
    } 
    numTimeAtSpeed_ = 0;
    atTargetSpeed_ = false;
    //return true;
    return false;
}

std::string SuperstructureController::GetControlPanelColor() {
    if(robot_->GetControlPanelGameData().length() > 0)
    {
        switch(robot_->GetControlPanelGameData()[0])
        {
            case 'B' :
                return "blue";
                break;
            case 'G' :
                return "green";
                break;
            case 'R' :
                return "red";
                break;
            case 'Y' :
                return "yellow";
                break;
            default :
                printf("error in receiving control panel color");
        }
    }
    return "error";
}

/*
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
                printf("this data is BAD BOIZ");
                break;
        }
    } else {
        printf("no data received yet");
        // no data received yet
    }
    nextState_ = kDefaultTeleop;
}

void SuperstructureController::ControlPanelFinalSpin() {
    if(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time and change to if
        robot_->SetControlPanelOutput(0.3); // fix power
    }
    robot_->SetControlPanelOutput(0.0);
}
*/

void SuperstructureController::RefreshShuffleboard(){
    manualRollerPower_ = rollerManualEntry_.GetDouble(manualRollerPower_);
    closeFlywheelVelocity_ = closeFlywheelEntry_.GetDouble(closeFlywheelVelocity_);
    elevatorFastPower_ = fastElevatorEntry_.GetDouble(elevatorFastPower_);
    elevatorSlowPower_ = slowElevatorEntry_.GetDouble(elevatorSlowPower_);
    indexFunnelPower_ = funnelEntry_.GetDouble(indexFunnelPower_);
    autoWristUpP_ = autoWristUpPEntry_.GetDouble(autoWristUpP_);
    autoWristDownP_ = autoWristDownPEntry_.GetDouble(autoWristDownP_);

    elevatorBottomLightSensorEntry_.SetBoolean(robot_->GetElevatorFeederLightSensorStatus());
    elevatorTopLightSensorEntry_.SetBoolean(robot_->GetElevatorLightSensorStatus());
    targetSpeedEntry_.SetBoolean(atTargetSpeed_);

    flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM); //rpm
    flywheelVelocityErrorEntry_.SetDouble(desiredFlywheelVelocity_-robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM);
    flywheelMotor1OutputEntry_.SetDouble(robot_->FlywheelMotor1Output());
    flywheelMotor2OutputEntry_.SetDouble(robot_->FlywheelMotor2Output());
    flywheelMotor1CurrentEntry_.SetDouble(robot_->GetFlywheelMotor1Current());
    flywheelMotor2CurrentEntry_.SetDouble(robot_->GetFlywheelMotor2Current());
    //printf("wrist angle: %f\n", currWristAngle_);
    intakeWristAngleEntry_.SetDouble(currWristAngle_);
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {
    intakeWristAngleEntry_.Delete();
    flywheelPEntry_.Delete();
    flywheelIEntry_.Delete();
    flywheelDEntry_.Delete();
    flywheelFEntry_.Delete();
    flywheelVelocityErrorEntry_.Delete();
    flywheelVelocityEntry_.Delete();
    elevatorBottomLightSensorEntry_.Delete();
    elevatorTopLightSensorEntry_.Delete();
    controlPanelColorEntry_.Delete();
}
