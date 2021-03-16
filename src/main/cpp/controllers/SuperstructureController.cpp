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
    // climbing
    climbElevatorUpPower_ = 0.8;//0.5; // fix
    climbElevatorDownPower_ = -0.4; // fix
    climbPowerDesired_ = 0.0; // needs to equal 0
    
    // flywheel/shooting
    desiredFlywheelVelocity_ = 0.0;
    closeFlywheelVelocity_ = 3100.0;//3550;
    flywheelResetTime_ = 2.0; // fix //why does this exist
    shootingIsDone_ = false;

    closePrepping_ = false;
    farPrepping_ = false;
    atTargetSpeed_ = false;
    numTimeAtSpeed_ = 0;

    // indexing and intaking
    elevatorFeederPower_ = 1.0; // fix
    elevatorSlowPower_ = 0.4; //fix
    elevatorFastPower_ = 0.4;//0.75; //fix
    indexFunnelPower_ = 0.3; // fix
    intakeRollersPower_ = 1.0;//0.5;
    manualRollerPower_ = 0.5;

    autoWristDownP_ = 0.07;
    autoWristUpP_ = 0.1;
    isManualRaisingWrist_ = false;
    desiredIntakeWristAngle_ = 150.0;//237.0; //down

    // control panel
    controlPanelPower_ = 0.5; // fix
    controlPanelCounter_ = 0;
    controlPanelStage2_ = false;
    controlPanelStage3_ = false;

    // states
    currSuperState_ = kDefaultTeleop;
    nextSuperState_ = kDefaultTeleop;
    currWristState_ = kRaising;
    nextWristState_ = kRaising;
    currHandlingState_ = kIndexing;
    nextHandlingState_ = kIndexing;
    
    // time and timeouts
    currTime_ = robot_->GetTime();
    startResetTime_ = currTime_;
    startElevatorTime_ = currTime_;
    shootPrepStartTime_ = currTime_;

    resetTimeout_ = 2.0;
    lowerElevatorTimeout_ = 2.0; //fix
    elevatorTimeout_ = 2.0;
    startIndexTime_ = currTime_-lowerElevatorTimeout_-1.0;
    startResetTime_ = currTime_-elevatorTimeout_-1.0;
    startRatchetTime_ = -1.0; // TODO check

    // shuffleboard
    flywheelVelocityEntry_ = flywheelPIDLayout_.Add("flywheel velocity", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelVelocityErrorEntry_ = flywheelPIDLayout_.Add("flywheel error", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    
    flywheelPEntry_ = flywheelPIDLayout_.Add("flywheel P", 0.35).GetEntry();
    flywheelIEntry_ = flywheelPIDLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = flywheelPIDLayout_.Add("flywheel D", 0.1).GetEntry();
    //flywheelFEntry_ = flywheelPIDLayout_.Add("flywheel FF", 1.0).GetEntry();
    flywheelMotor1OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 output", robot_->FlywheelMotor1Output()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor2OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 output", robot_->FlywheelMotor2Output()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor1CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 current", robot_->GetFlywheelMotor1Current()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    flywheelMotor2CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 current", robot_->GetFlywheelMotor2Current()).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
    closeFlywheelEntry_ = powerLayout_.Add("close flywheel velocity", closeFlywheelVelocity_).GetEntry();
    targetSpeedEntry_ = flywheelPIDLayout_.Add("target speed", atTargetSpeed_).GetEntry();

    autoWristEntry_ = manualOverrideLayout_.Add("auto wrist", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).GetEntry();
    autoWristDownPEntry_ = robot_->GetPIDTab().Add("wrist down p", autoWristDownP_).GetEntry();
    autoWristUpPEntry_ = robot_->GetPIDTab().Add("wrist up p", autoWristUpP_).GetEntry();
    intakeWristAngleEntry_ = sensorsLayout_.Add("intake wrist angle", 0.0).GetEntry();
    rollerManualEntry_ = powerLayout_.Add("manual rollers", manualRollerPower_).GetEntry();

    slowElevatorEntry_ = powerLayout_.Add("slow elevator", elevatorSlowPower_).GetEntry();
    fastElevatorEntry_ = powerLayout_.Add("fast elevator", elevatorFastPower_).GetEntry();

    funnelEntry_ = powerLayout_.Add("funnel", indexFunnelPower_).GetEntry();
    
    climbElevatorUpEntry_ = robot_->GetSuperstructureTab().Add("Elevator Up Power", climbElevatorUpPower_).GetEntry();
	climbElevatorDownEntry_ = robot_->GetSuperstructureTab().Add("Elevator Down Power", climbElevatorDownPower_).GetEntry();

    //TODO make timeout

    // sensors
    elevatorTopLightSensorEntry_ = sensorsLayout_.Add("top elevator", false).GetEntry();
    elevatorBottomLightSensorEntry_ = sensorsLayout_.Add("bottom elevator", false).GetEntry();

    // control panel
    controlPanelColorEntry_ = robot_->GetFunctionalityTab().Add("control panel color", "").GetEntry();
    controlPanelColorEntry_.SetString(GetControlPanelColor());
    //TODO make timeout

    flywheelRPMconstEntry_ = robot_->GetDriverTab().Add("Flywheel rpm C", 0.0).GetEntry();

    printf("end of superstructure controller constructor\n");

}

//auto init
void SuperstructureController::AutoInit(){
    shootingIsDone_ = false;
    currHandlingState_ = kIndexing; // TODO AHHHH might break. Possible error
    nextHandlingState_ = kIndexing;
}

//teleop and auto init
void SuperstructureController::Reset() { // might not need this
    closePrepping_ = false;
    farPrepping_ = false;

    currSuperState_ = kDefaultTeleop;
    nextSuperState_ = kDefaultTeleop;
    currHandlingState_ = kIndexing;
    nextHandlingState_ = kIndexing;
    currWristState_ = kRaising;
    nextWristState_ = kRaising;

    flywheelPFac_ = flywheelPEntry_.GetDouble(0.35);
    flywheelIFac_ = flywheelIEntry_.GetDouble(0.0);
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.1);
    //flywheelFFac_ = flywheelFEntry_.GetDouble(0.0);
    FlywheelPIDControllerUpdate();

    
}

// auto wrist and manual wrist update, sets wrist power
void SuperstructureController::WristUpdate(bool isAuto){
    //auto wrist 
    double intakeWristOutput = 0.0;
    double intakeRollersOutput = 0.0;

    if (autoWristEntry_.GetBoolean(true)){
        currWristAngle_ = robot_->GetIntakeWristAngle(); // might not need?
        switch (currWristState_){
            case kRaising:
                if (currWristAngle_ > 10.0) {
                    intakeWristOutput = autoWristUpP_*(0.0-currWristAngle_);
                }

                break;
            case kLowering:
                if (currWristAngle_ < desiredIntakeWristAngle_-45.0) {
                    intakeWristOutput = autoWristDownP_*(desiredIntakeWristAngle_-currWristAngle_);
                }
                intakeRollersOutput = intakeRollersPower_;

                break;
            default:
                printf("ERROR: no state in wrist controller \n");
        }
        currWristState_ = nextWristState_;

    }

    //manual wrist override
    if (isManualRaisingWrist_ && !humanControl_->GetDesired(ControlBoard::Buttons::kWristUpButton)){ //was just raising wrist
        robot_->ResetWristAngle();
    }
    isManualRaisingWrist_ = false;

    if (humanControl_->GetDesired(ControlBoard::Buttons::kWristUpButton)){
        intakeWristOutput = -0.5;
        isManualRaisingWrist_ = true;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kWristDownButton)){
        intakeWristOutput = 0.5;
    }

    if (humanControl_->GetDesired(ControlBoard::Buttons::kReverseRollersButton)){
        intakeRollersOutput = -0.5;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kRunRollersButton)){
        intakeRollersOutput = manualRollerPower_;
    }

    // if auto wrist update and the rollers will run, set rollers output to full power
    if (isAuto && fabs(intakeRollersOutput) > 0.1){
        intakeRollersOutput = 1.0;
    } 

    // Utah fix DO NOT DELETE
    // if(!isAuto){
    //     if(((int)(currTime_*5.0))%6 == 0){
    //         intakeRollersOutput *= -1.0;
    //     }
    //     intakeRollersOutput *= 0.5;
    // }

    robot_->SetIntakeWristOutput(intakeWristOutput);
    robot_->SetIntakeRollersOutput(intakeRollersOutput);

}

// updates buttons and prepares for shooting in auto
void SuperstructureController::UpdatePrep(bool isAuto){
    if (!isAuto){
        UpdateButtons(); //moved button/state code into that function B)
    }  else if (!farPrepping_ && !closePrepping_ && currHandlingState_ != kShooting){ //farPrepping_ biconditional kPrepping :(((((
        desiredFlywheelVelocity_ = 0.0;
        SetFlywheelPowerDesired(0.0);
        robot_->SetControlModeVelocity(0.0);
    }
        //SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION

        //MOVED FLYWHEEL VELOCITY SETTING TO SetPreppingState()
    
}

// START OF NEW STATE MACHINE!! - DO NOT TOUCH PLS
void SuperstructureController::Update(bool isAuto){
    currTime_ = robot_->GetTime(); // may or may not be necessary
    RefreshShuffleboard();

    switch(currSuperState_){ 
        case kDefaultTeleop:
            if (!isAuto){
                CheckClimbDesired();
                CheckControlPanelDesired();
            }
            
            // should these be inside or outside power cell handling
            //CheckControlPanelDesired(); // might have to move out of the DefaultTeleop
            //CheckClimbDesired();

            UpdatePrep(isAuto);
            WristUpdate(isAuto);

            IndexPrep(isAuto);

            //in current code: true is arm down and false is arm up
            //TODO ADD CLIMBING AND SPINNER SEMIAUTO
            switch(currHandlingState_){
                case kIntaking:
                    Intaking();
                    break;
                case kIndexing:
                    Indexing();
                    break;
                case kShooting:
                    shootingIsDone_ = Shooting(isAuto_);
                    break;
                case kResetting:
                    Resetting();
                    break;
                case kUndoElevator:
                    UndoElevator();
                    break;
                case kManualFunnelFeederElevator:
                    ManualFunnelFeederElevator();
                    break;
                default:
                    printf("ERROR: no state in Power Cell Handling \n");
            }

            currHandlingState_ = nextHandlingState_;
            
            break;
        case kControlPanel:
            printf("control panel state \n");

            robot_->SetControlPanelOutput(controlPanelPower_);
            if (!humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
                robot_->SetControlPanelOutput(0.0);
                nextSuperState_ = kDefaultTeleop;
            }
            break;
        case kClimbing:
            printf("climbing state \n");

            robot_->EngageClimberRatchet();
            // giving the ratchet time to actually move
            if (currTime_-startRatchetTime_ >= 0.5){
                if (humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton)){
                    robot_->SetRightClimberElevatorOutput(climbElevatorUpPower_);
                } else if (humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton)){
                    robot_->SetRightClimberElevatorOutput(climbElevatorDownPower_);
                } else {
                    robot_->SetRightClimberElevatorOutput(0.0);
                }   

                if (humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton)){
                    robot_->SetLeftClimberElevatorOutput(climbElevatorUpPower_);
                } else if (humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)){
                    robot_->SetLeftClimberElevatorOutput(climbElevatorDownPower_);
                } else {
                    robot_->SetLeftClimberElevatorOutput(0.0);
                }
            } else {
                robot_->SetLeftClimberElevatorOutput(0.0);
                robot_->SetRightClimberElevatorOutput(0.0);
            }
            
            if (!humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton) &&
            !humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton) &&
            !humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton) &&
            !humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)){
                nextSuperState_ = kDefaultTeleop; // verify that it should be next state
                robot_->DisengageClimberRatchet();
            }
            break;
        default:
            printf("ERROR: no state in superstructure controller\n");
            desiredFlywheelVelocity_ = 0.0;
            SetFlywheelPowerDesired(0.0);
            robot_->SetControlModeVelocity(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            nextWristState_ = kRaising;

    }
    currSuperState_ = nextSuperState_;
}

// updates states according to buttons
void SuperstructureController::UpdateButtons(){
    // determining next handling state
    if (humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){ // intaking
        nextHandlingState_ = kIntaking;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton)){ // shooting, TODO remove this, doesn't do anything????
        if (nextHandlingState_!=kShooting){ // get start time
           startIndexTime_ = currTime_;
        }
        //printf("REACHED THIS PART OF CODE\n");
        nextHandlingState_ = kShooting; 
    } else if (nextHandlingState_ == kShooting){ //shooting to decide not to shoot. kShootingButton is not pressed (case taken care of in previous else if statement)
        nextHandlingState_ = kResetting;
    } else if (nextHandlingState_ != kResetting){ //not intaking, shooting, or resetting, only option is indexing or prepping (also includes indexing)
        nextHandlingState_ = kIndexing;
    }

    PowerCellHandlingState prevHandlingState = nextHandlingState_; 

    // undo elevator button
    if (humanControl_->GetDesired(ControlBoard::Buttons::kUndoElevatorButton)){
        nextHandlingState_ = kUndoElevator;
    } else if (nextHandlingState_ == kUndoElevator && !humanControl_->GetDesired(ControlBoard::Buttons::kUndoElevatorButton)) {
        nextHandlingState_ = prevHandlingState; 
    }

    // not currently shooting but wants to shoot, shooting initializations 
    if (nextHandlingState_ != kShooting){
        if (humanControl_->GetDesired(ControlBoard::Buttons::kShootClosePrepButton)){
            // close shot prep
            if (!closePrepping_){
                // start close prep shooting
                shootPrepStartTime_ = currTime_;
            }
            // in close prep shooting
            desiredFlywheelVelocity_ = closeFlywheelVelocity_;
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->DisengageFlywheelHood();
            closePrepping_ = true;
            farPrepping_ = false;

        //testing purposes
        } else if (humanControl_->GetDesired(ControlBoard::Buttons::kTestFarShootButton)){
            // far shot prep
            if (!closePrepping_){
                // start close prep shooting
                shootPrepStartTime_ = currTime_;
            }
            // in close prep shooting
            desiredFlywheelVelocity_ = closeFlywheelVelocity_;
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->EngageFlywheelHood();
            closePrepping_ = true;
            farPrepping_ = false;

        } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootFarPrepButton)){
            // far prep
            if (!farPrepping_){
                // start far prep shooting
                shootPrepStartTime_ = currTime_;
            }
            desiredFlywheelVelocity_ = CalculateFlywheelVelocityDesired();
            SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION
            robot_->EngageFlywheelHood(); //TODO add if distance > x
            closePrepping_ = false;
            farPrepping_ = true;
        } else {
            // not prepping, stop flywheel
            closePrepping_ = false;
            farPrepping_ = false;
            desiredFlywheelVelocity_ = 0.0;
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->SetControlModeVelocity(0.0);
            robot_->DisengageFlywheelHood();
        }
    } 

}

// checks if control panel button is pressed, sets next super state if so
void SuperstructureController::CheckControlPanelDesired(){
    SuperstructureState previousState = currSuperState_;
    if (humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
        controlPanelStage2_ = true;
        controlPanelStage3_ = false;
        nextSuperState_ = kControlPanel;
    } 
}

// checks if climb buttons are pressed, if so, set next super state
void SuperstructureController::CheckClimbDesired(){
    SuperstructureState previousState = currSuperState_;
    if (humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorUpButton) ||
    humanControl_->GetDesired(ControlBoard::Buttons::kClimbRightElevatorDownButton) ||
    humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorUpButton) ||
    humanControl_->GetDesired(ControlBoard::Buttons::kClimbLeftElevatorDownButton)) {
        nextSuperState_ = kClimbing;
        startRatchetTime_ = robot_->GetTime();
    }
}

// prepping for indexing, setting timeout start times
void SuperstructureController::IndexPrep(bool isAuto){
    bottomSensor_ = robot_->GetElevatorFeederLightSensorStatus();
    topSensor_ = robot_->GetElevatorLightSensorStatus();
    
    if (topSensor_){
        startElevatorTime_ = currTime_;
    }
    if (bottomSensor_){
        startIndexTime_ = currTime_;
    }

    if (isAuto){
        tTimeout_ = currTime_-startElevatorTime_ > 0.2;
    } else {
        tTimeout_ = currTime_-startElevatorTime_ > elevatorTimeout_;
    }
    bTimeout_ = currTime_-startIndexTime_ > lowerElevatorTimeout_;
}

void SuperstructureController::Intaking(){
    nextWristState_ = kLowering;
    IndexUpdate();
}

void SuperstructureController::Indexing(){
    IndexUpdate();
    nextWristState_ = kRaising;
}

bool SuperstructureController::Shooting(bool isAuto) {

    SetFlywheelPowerDesired(desiredFlywheelVelocity_);

    //raise elevator if not at speed, OR nothing at top and not timed out at bottom
    if (IsFlywheelAtSpeed(desiredFlywheelVelocity_) || (!topSensor_ && !bTimeout_)){
        robot_->SetElevatorOutput(elevatorSlowPower_);
    } else {                                                                                          
        robot_->SetElevatorOutput(0.0);
    }

    if (!bottomSensor_ && !bTimeout_){
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    } else {
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
    }

    nextWristState_ = kRaising;

    if (tTimeout_ && bTimeout_){
        robot_->SetControlModeVelocity(0.0);
        nextHandlingState_ = kIndexing;
        return true;
    }
    return false;

}

// resets by bringing down elevator, raising the wrist, and stopping everything else
void SuperstructureController::Resetting() {
    printf("in kResetting\n");
    desiredFlywheelVelocity_ = 0.0;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    robot_->SetControlModeVelocity(0.0);

    if (!bottomSensor_ && currTime_-startResetTime_ <= resetTimeout_){
        robot_->SetElevatorOutput(-elevatorFastPower_); //bring down elevator
    } else {
        printf("elevator at 0\n");
        robot_->SetElevatorOutput(0.0);
        nextHandlingState_ = kIndexing;
    }

    robot_->SetIndexFunnelOutput(0.0);
    robot_->SetElevatorFeederOutput(0.0);
    nextWristState_ = kRaising;
}

// manual undo elevator
void SuperstructureController::UndoElevator(){
    robot_->SetElevatorOutput(-elevatorSlowPower_);
    robot_->SetElevatorFeederOutput(-elevatorFeederPower_);

    // non utah fix
    robot_->SetIndexFunnelOutput(indexFunnelPower_);

    // Utah fixes DO NOT DELETE!
    // if(((int)currTime_)%4 == 0){
    //     robot_->SetIndexFunnelOutput(-indexFunnelPower_); //TODO PUT BACK IN
    // } else {
    //     robot_->SetIndexFunnelOutput(indexFunnelPower_);
    // }

    robot_->SetFlywheelOutput(-elevatorSlowPower_); // might need to adjust
}

// manual run everything, funnel, feeder, elevator
void SuperstructureController::ManualFunnelFeederElevator(){
    robot_->SetElevatorOutput(elevatorSlowPower_);
    robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    robot_->SetIndexFunnelOutput(indexFunnelPower_);
}

// indexing
void SuperstructureController::IndexUpdate(){

    // only run elevator if !t && b
    if (!topSensor_ && bottomSensor_){
        robot_->SetElevatorOutput(elevatorFastPower_);
    } else {
        robot_->SetElevatorOutput(0.0);
    }

    //4 ball elevator:  run feeder no matter what unless timeout or t && b.
    //if (!(topSensor_ && bottomSensor_) && !bTimeout_){
    //    robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    //} else {
    //    robot_->SetElevatorFeederOutput(0.0);
    //}

    // run funnel if !b
    if (!bottomSensor_ && (!bTimeout_ || currHandlingState_ == kIntaking)){

        // UTAH FIX DO NOT DELETE
        // if(((int)currTime_)%4 == 0){
        //     robot_->SetIndexFunnelOutput(-indexFunnelPower_); //TODO PUT BACK IN
        // } else {
        //     robot_->SetIndexFunnelOutput(indexFunnelPower_);
        // }

        robot_->SetIndexFunnelOutput(indexFunnelPower_);
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
        
    } else {
        robot_->SetIndexFunnelOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
    }

    // Original logic (old funnel) DO NOT DELETE (utah fix)
    // //control top
    // if(!topSensor_ && bottomSensor_){
    //     robot_->SetElevatorOutput(elevatorFastPower_);
    // } else {
    //     robot_->SetElevatorOutput(0.0);
    // }
    
    // //control bottom
    // if(!bottomSensor_ && (!bTimeout_ || currHandlingState_ == kIntaking)){
    //     if(((int)currTime_)%4 == 0){
    //         robot_->SetIndexFunnelOutput(-indexFunnelPower_); //TODO PUT BACK IN
    //     } else {
    //         robot_->SetIndexFunnelOutput(indexFunnelPower_);
    //     }
    //     robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    // } else {
    //     robot_->SetIndexFunnelOutput(0.0);
    //     robot_->SetElevatorFeederOutput(0.0);       
    // }

}

bool SuperstructureController::GetShootingIsDone(){
    return shootingIsDone_;
}

void SuperstructureController::SetShootingState(double autoVelocity){
    //robot_->SetLight(true);
    //distanceToTarget_ = robot_->GetDistance();
    //desiredFlywheelVelocity_ = (distanceToTarget_+1827.19)/0.547; //velocity from distance, using desmos
    desiredFlywheelVelocity_=autoVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    startElevatorTime_ = currTime_;
    startIndexTime_ = currTime_;
    nextWristState_ = kRaising; //resetting whatever intake did
    nextHandlingState_ = kShooting;
}

void SuperstructureController::SetIndexingState(){
    nextWristState_ = kRaising; //resetting whatever intake did
    nextHandlingState_ = kIndexing;
    if (!closePrepping_ && !farPrepping_){
        robot_->SetControlModeVelocity(0.0);
    }
}

void SuperstructureController::SetIntakingState(){
    nextWristState_ = kLowering;
    nextHandlingState_ = kIntaking;
    if (!closePrepping_ && !farPrepping_){
        robot_->SetControlModeVelocity(0.0);
    }
}

void SuperstructureController::SetPreppingState(double desiredVelocity){ //starts warming up shooter B)
    //distanceToTarget_ = robot_->GetDistance();
    //desiredVelocity = (distanceToTarget_+1827.19)/0.547; //velocity from distance, using desmos
    nextWristState_ = kRaising; //resetting whatever intake did
    if (!farPrepping_){ 
        shootPrepStartTime_ = robot_->GetTime(); //TODO FIX
    }
    farPrepping_ = true;
    closePrepping_ = false;
    desiredFlywheelVelocity_ = desiredVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
}


void SuperstructureController::FlywheelPIDControllerUpdate() {
    flywheelFFac_ = RatioFlywheel();
    robot_->ConfigFlywheelPID(flywheelPFac_, flywheelIFac_, flywheelDFac_);
}

double SuperstructureController::RatioFlywheel(){
    return MAX_FALCON_RPM*robot_->GetVoltage()/RATIO_BATTERY_VOLTAGE;
}

// uses inches
double SuperstructureController::CalculateFlywheelVelocityDesired() {
    double shotDistance = sqrt(pow(robot_->GetDistance()*12.0, 2.0) - pow(60.0, 2.0)) + 6.0; //all in inches
    //nasa is -320
    //into shot but slightly too high is -225
    //match 43 is -300
    double desiredVelocity = 5.58494*shotDistance + 2966.29 - 225.0 + flywheelRPMconst_;

    return desiredVelocity;
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

bool SuperstructureController::IsFlywheelAtSpeed(double rpm){
    printf("CURRENT SPEED IS %f\n", robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM);
    if (robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM > rpm-50 && 
        robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM < rpm+50){
        numTimeAtSpeed_++;
        if (numTimeAtSpeed_ >= 5){
            atTargetSpeed_ = true;
            printf("REACHED SPEED\n");
        } else {
            //numTimeAtSpeed_ = 0;
            atTargetSpeed_ = false;
        }
        //atTargetSpeed_ = true;
    } else {
        numTimeAtSpeed_ = 0;
        atTargetSpeed_ = false;
    }
    // TODO for operator to decide when she wants to shoot
    return atTargetSpeed_;
}

bool SuperstructureController::GetIsPrepping(){
    return (farPrepping_ || closePrepping_);
}

std::string SuperstructureController::GetControlPanelColor() {
    if (robot_->GetControlPanelGameData().length() > 0)
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
    nextSuperState_ = kDefaultTeleop;
}

void SuperstructureController::ControlPanelFinalSpin() {
    if(robot_->GetTime()-initialControlPanelTime_ < 2.0) { // fix time and change to if
        robot_->SetControlPanelOutput(0.3); // fix power
    }
    robot_->SetControlPanelOutput(0.0);
}
*/

void SuperstructureController::RefreshShuffleboard(){
    flywheelRPMconst_ = flywheelRPMconstEntry_.GetDouble(0.0);

    manualRollerPower_ = rollerManualEntry_.GetDouble(manualRollerPower_);
    autoWristUpP_ = autoWristUpPEntry_.GetDouble(autoWristUpP_);
    autoWristDownP_ = autoWristDownPEntry_.GetDouble(autoWristDownP_);
    intakeWristAngleEntry_.SetDouble(currWristAngle_);

    closeFlywheelVelocity_ = closeFlywheelEntry_.GetDouble(closeFlywheelVelocity_);

    elevatorFastPower_ = fastElevatorEntry_.GetDouble(elevatorFastPower_);
    elevatorSlowPower_ = slowElevatorEntry_.GetDouble(elevatorSlowPower_);
    indexFunnelPower_ = funnelEntry_.GetDouble(indexFunnelPower_);

    climbElevatorUpPower_ = climbElevatorUpEntry_.GetDouble(climbElevatorUpPower_);
    climbElevatorDownPower_ = climbElevatorDownEntry_.GetDouble(climbElevatorDownPower_);
    
    elevatorBottomLightSensorEntry_.SetBoolean(robot_->GetElevatorFeederLightSensorStatus());
    elevatorTopLightSensorEntry_.SetBoolean(robot_->GetElevatorLightSensorStatus());
    targetSpeedEntry_.SetBoolean(atTargetSpeed_);

    flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM); //rpm
    flywheelVelocityErrorEntry_.SetDouble(desiredFlywheelVelocity_-robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM);
    flywheelMotor1OutputEntry_.SetDouble(robot_->FlywheelMotor1Output());
    flywheelMotor2OutputEntry_.SetDouble(robot_->FlywheelMotor2Output());
    flywheelMotor1CurrentEntry_.SetDouble(robot_->GetFlywheelMotor1Current());
    flywheelMotor2CurrentEntry_.SetDouble(robot_->GetFlywheelMotor2Current());
    
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {
    flywheelRPMconstEntry_.Delete();
    intakeWristAngleEntry_.Delete();
    
    flywheelPEntry_.Delete();
    flywheelIEntry_.Delete();
    flywheelDEntry_.Delete();
    //flywheelFEntry_.Delete();

    flywheelVelocityErrorEntry_.Delete();
    flywheelVelocityEntry_.Delete();
    flywheelMotor1OutputEntry_.Delete();
    flywheelMotor2OutputEntry_.Delete();
    flywheelMotor1CurrentEntry_.Delete();
    flywheelMotor2CurrentEntry_.Delete();
    
    elevatorBottomLightSensorEntry_.Delete();
    elevatorTopLightSensorEntry_.Delete();

    autoWristEntry_.Delete();
    autoWristUpPEntry_.Delete();
    autoWristDownPEntry_.Delete();

    climbElevatorUpEntry_.Delete();
    climbElevatorDownEntry_.Delete();

    controlPanelColorEntry_.Delete();

    slowElevatorEntry_.Delete();
    fastElevatorEntry_.Delete();
    funnelEntry_.Delete();
    rollerManualEntry_.Delete();
    closeFlywheelEntry_.Delete();
    targetSpeedEntry_.Delete();
}
