/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. Tb      code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
#include <math.h>
#define SHUFFLEBOARDCONTROLS

/**
 * Constructor for SuperstructureController
 * @param robot a RobotModel
 * @param humanControl a ControlBoard
 */
SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) :
    flywheelPIDLayout_(robot->GetSuperstructureTab().GetLayout("Flywheel", "List Layout").WithPosition(0, 0)),
    sensorsLayout_(robot->GetSuperstructureTab().GetLayout("Sensors", "List Layout").WithPosition(0, 1)),

    manualOverrideLayout_(robot->GetModeTab().GetLayout("climb override", "List Layout").WithPosition(1,1)),
    powerLayout_(robot->GetSuperstructureTab().GetLayout("power control", "List Layout").WithPosition(3, 0)),
    currentLayout_(robot->GetSuperstructureTab().GetLayout("motor current", "List Layout").WithPosition(4, 0)),
    timeoutsLayout_(robot->GetSuperstructureTab().GetLayout("jammed timeouts control", "List Layout").WithPosition(5,0))
    {
    
    robot_ = robot;
    humanControl_ = humanControl; 

    // climbing
    climbElevatorUpPower_ = 0.8;
    climbElevatorDownPower_ = -0.4;
    climbPowerDesired_ = 0.0; // needs to equal 0
    
    // flywheel/shooting
    desiredFlywheelVelocity_ = 0.0;
    closeFlywheelVelocity_ = 3100.0;
    flywheelResetTime_ = 2.0;
    shootingIsDone_ = false;

    closePrepping_ = false;
    farPrepping_ = false;
    atTargetSpeed_ = false;
    numTimeAtSpeed_ = 0;
    highFlywheelTolerance_ = 0.06;
    flywheelPercentAdjustment_ = 101.0; // percent!

    // indexing and intaking

    // Values for Chezy Champs/5 ball indexing
    elevatorFeederPower_ = 0.8;
    elevatorSlowFeederPower_ = 0.6;
    elevatorSlowPower_ = 0.5;
    elevatorFastPower_ = 0.5;
    indexFunnelSlowPower_ = 0.55;
    indexFunnelPower_ = 0.50;
    intakeRollersPower_ = 0.95;
    intakeSlowRollersPower_ = 0.7;
    manualRollerPower_ = 0.5;

    autoWristDownP_ = 0.07;
    autoWristUpP_ = 0.1;
    isManualRaisingWrist_ = false;
    desiredIntakeWristAngle_ = 150.0; 

    // control panel
    controlPanelPower_ = 0.5;
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
    currIndexLogicState_ = kIdle;
    nextIndexLogicState_ = kIdle;
    
    
    // time and timeouts
    currTime_ = robot_->GetTime();
    startResetTime_ = currTime_;
    startIndexingTime_ = currTime_;
    startElevatorTime_ = currTime_;
    shootPrepStartTime_ = currTime_;

    resetTimeout_ = 2.0;
    lowerElevatorTimeout_ = 2.0; 
    resetElevatorTimeout_ = 5.0; // time before bringing down the balls when t && !b
    startResetElevatorTime_ = currTime_;
    elevatorTimeout_ = 2.0;
    startIndexTime_ = currTime_-lowerElevatorTimeout_-1.0;
    startReIndexTime_ = currTime_;
    startResetTime_ = currTime_-elevatorTimeout_-1.0;
    startRatchetTime_ = -1.0; 
    jammedStartTimeout_ = currTime_;
    jammedTimeout_ = 0.6;

    currJammed_ = false;

    motorCurrentLimit_ = 25.0;

    // indexing logic
    isBallIncoming_ = false;  

    // shuffleboard

#ifdef SHUFFLEBOARDCONTROLS
    flywheelVelocityEntry_ = flywheelPIDLayout_.Add("flywheel velocity", 0.0).GetEntry();
    flywheelVelocityErrorEntry_ = flywheelPIDLayout_.Add("flywheel error", 0.0).GetEntry();
#endif
    flywheelPEntry_ = flywheelPIDLayout_.Add("flywheel P", 0.35).GetEntry();
    flywheelIEntry_ = flywheelPIDLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = flywheelPIDLayout_.Add("flywheel D", 0.0).GetEntry();
    flywheelFEntry_ = flywheelPIDLayout_.Add("flywheel FF", 1.0).GetEntry();
#ifdef SHUFFLEBOARDCONTROLS
    flywheelMotor1OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 output", robot_->FlywheelMotor1Output()).GetEntry();
    flywheelMotor2OutputEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 output", robot_->FlywheelMotor2Output()).GetEntry();
    flywheelMotor1CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 1 current", robot_->GetFlywheelMotor1Current()).GetEntry();
    flywheelMotor2CurrentEntry_ = flywheelPIDLayout_.Add("flywheel motor 2 current", robot_->GetFlywheelMotor2Current()).GetEntry();
#endif

    targetSpeedEntry_ = flywheelPIDLayout_.Add("target speed", atTargetSpeed_).GetEntry();

    closeFlywheelEntry_ = powerLayout_.Add("close flywheel velocity", closeFlywheelVelocity_).GetEntry();

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

    // sensors
    elevatorTopLightSensorEntry_ = sensorsLayout_.Add("top elevator", false).GetEntry();
    elevatorBottomLightSensorEntry_ = sensorsLayout_.Add("bottom elevator", false).GetEntry();
    funnelLightSensorEntry_ = sensorsLayout_.Add("funnel sensor", false).GetEntry();

    // control panel
    controlPanelColorEntry_ = robot_->GetFunctionalityTab().Add("control panel color", "").GetEntry();
    controlPanelColorEntry_.SetString(GetControlPanelColor());

    flywheelRPMconstEntry_ = robot_->GetDriverTab().Add("Flywheel rpm C", 0.0).GetEntry();
    highFlywheelToleranceEntry_ = robot_->GetSuperstructureTab().Add("High Flywheel Tolerance (0.02 = 2 percent)", 0.06).GetEntry();
    flywheelPercentAdjustmentEntry_ = robot_->GetSuperstructureTab().Add("Percent Flywheel Adjustment", 100.0).GetEntry();

    // funnel and feeder 
    funnelRightMotorEntry_ = currentLayout_.Add("Funnel Right Motor", 0.0).GetEntry();
    funnelLeftMotorEntry_ = currentLayout_.Add("Funnel Left Motor", 0.0).GetEntry();
    feederMotorEntry_ = currentLayout_.Add("Feeder Motor", 0.0).GetEntry();

    jammedTimeoutEntry_ = timeoutsLayout_.Add("Jammed Timeout", jammedTimeout_).GetEntry();
    currentLimitEntry_ = timeoutsLayout_.Add("Motor Current Limit", motorCurrentLimit_).GetEntry();
    
    printf("end of superstructure controller constructor\n");

}

/**
 * Auto init
 */
void SuperstructureController::AutoInit(){
    shootingIsDone_ = false;
    startReIndexTime_ = currTime_;
    startIndexingTime_ = currTime_;
}

/**
 * Teleop init
 */
void SuperstructureController::TeleopInit(){
    startReIndexTime_ = currTime_;
    startIndexingTime_ = currTime_;
}

/**
 * Resets for both teleop and auto
 */
void SuperstructureController::Reset() {

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
    flywheelDFac_ = flywheelDEntry_.GetDouble(0.0);
    flywheelFFac_ = flywheelFEntry_.GetDouble(0.0);
    FlywheelPIDControllerUpdate();
}

/** 
 * Auto wrist and manual wrist update, sets wrist power
 * @param isAuto a boolean
 */
void SuperstructureController::WristUpdate(bool isAuto){
    //auto wrist 
    double intakeWristOutput = 0.0;
    double intakeRollersOutput = 0.0;

    if (autoWristEntry_.GetBoolean(true)){
        currWristAngle_ = robot_->GetIntakeWristAngle(); 
        switch (currWristState_){
            case kRaising: //if climbing, raise wrist
                if (currWristAngle_ > 10.0) {
                    intakeWristOutput = autoWristUpP_*(0.0-currWristAngle_);
                }
                break;
            case kLowering:
                if (currWristAngle_ < desiredIntakeWristAngle_-45.0) {
                    intakeWristOutput = autoWristDownP_*(desiredIntakeWristAngle_-currWristAngle_);
                }
                intakeRollersOutput = intakeRollersPower_;

                printf("SPEED: %f", robot_->GetLeftVelocity());

                if ((robot_->GetLeftVelocity() + robot_->GetRightVelocity())/2.0 < 0.3){
                    intakeRollersOutput = intakeSlowRollersPower_;
                }

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
    
    robot_->SetIntakeWristOutput(intakeWristOutput);
    robot_->SetIntakeRollersOutput(intakeRollersOutput);

}

/** 
 * Updates buttons and prepares for shooting in auto
 * @param isAuto a boolean
 */
void SuperstructureController::UpdatePrep(bool isAuto){
    if (!isAuto){
        UpdateButtons();
    } else if (!farPrepping_ && !closePrepping_ && currHandlingState_ != kShooting){
        desiredFlywheelVelocity_ = 0.0;
        SetFlywheelPowerDesired(0.0);
        robot_->SetControlModeVelocity(0.0);
    }
    
}

/** 
 * State machine!
 * @param isAuto a boolean
 */
void SuperstructureController::Update(bool isAuto){
    currTime_ = robot_->GetTime();
    RefreshShuffleboard();

    switch(currSuperState_){ 
        case kDefaultTeleop: // also auto
            if (!isAuto){
                CheckClimbDesired();
                CheckControlPanelDesired();
            }

            UpdatePrep(isAuto);
            WristUpdate(isAuto);

            IndexPrep(isAuto);

            // in current code: true is arm down and false is arm up
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
                nextSuperState_ = kDefaultTeleop; 
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
            // printf("HERE: 7");
            currWristState_ = kRaising;
            nextWristState_ = kRaising;

    }

    currSuperState_ = nextSuperState_;
}

/** 
 * Updates states according to buttons
 */
void SuperstructureController::UpdateButtons(){
    // determining next handling state
    if (humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){ // intaking
        nextHandlingState_ = kIntaking;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton)){ // shooting
        if (nextHandlingState_!=kShooting){ // get start time
           startIndexTime_ = currTime_;
        }
        nextHandlingState_ = kShooting; 
    } else if (nextHandlingState_ == kShooting){ //shooting to decide not to shoot. kShootingButton is not pressed (case taken care of in previous else if statement)
        nextHandlingState_ = kResetting;
    } else if (nextHandlingState_ != kResetting){ // not intaking, shooting, or resetting, only option is indexing or prepping (also includes indexing)
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
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            robot_->EngageFlywheelHood();
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

/**
 * Checks if control panel button is pressed, sets next super state if so
 */ 
void SuperstructureController::CheckControlPanelDesired(){
    SuperstructureState previousState = currSuperState_;
    // if button is pressed
    if (humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelButton)){
        controlPanelStage2_ = true;
        controlPanelStage3_ = false;
        nextSuperState_ = kControlPanel;
    } 
}

/** 
 * Checks if climb buttons are pressed, if so, set next super state
 */ 
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

/**
 * Prepping for indexing, setting timeout start times
 * @param isAuto a boolean
 */ 
void SuperstructureController::IndexPrep(bool isAuto){
    bottomSensor_ = robot_->GetElevatorFeederLightSensorStatus();
    topSensor_ = robot_->GetElevatorLightSensorStatus();
    funnelSensor_ = robot_->GetFunnelLightSensorStatus();
    
    // start time for timeout
    if (topSensor_){
        startElevatorTime_ = currTime_;
    }

    if (bottomSensor_ && !isAuto){
        startIndexTime_ = currTime_;
    }

    if (isAuto){
        tTimeout_ = currTime_-startElevatorTime_ > 0.2;
    } else {
        tTimeout_ = currTime_-startElevatorTime_ > elevatorTimeout_;
    }
    bTimeout_ = currTime_-startIndexTime_ > lowerElevatorTimeout_;
}

/**
 * In intaking state, lower wrist and index
 */ 
void SuperstructureController::Intaking(){
    nextWristState_ = kLowering;
    IndexUpdate();
}

/**
 * In indexing state, raise wrist and index
 */ 
void SuperstructureController::Indexing(){
    IndexUpdate();
    nextWristState_ = kRaising;
}

/**
 * Shooting state
 * @param isAuto a boolean
 * @return true if done
 */ 
bool SuperstructureController::Shooting(bool isAuto) {

    SetFlywheelPowerDesired(desiredFlywheelVelocity_);

    // raise elevator if not at speed, OR nothing at top and not timed out at bottom
    if (IsFlywheelAtSpeed(desiredFlywheelVelocity_) || (!topSensor_ && !bTimeout_)){
        // nothing at the top OR at desired speed, move elevator up
        robot_->SetElevatorOutput(0.9);
        robot_->SetElevatorFeederOutput(0.55);

        // also move funnel and feeder if either the bottom  or funnel sensors see something
        if (!bottomSensor_){
            robot_->SetIndexFunnelOutput(0.4);
        } else{
            robot_->SetIndexFunnelOutput(0.0);
        }
    } else {                                                                                          
        robot_->SetElevatorOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
        robot_->SetIndexFunnelOutput(0.0);
    }

    nextWristState_ = kRaising;

    if (tTimeout_ && bTimeout_){
        robot_->SetControlModeVelocity(0.0);
        nextHandlingState_ = kIndexing;
        return true;
    }
    return false;

}

/**
 * Resets by bringing down elevator, raising the wrist, and stopping everything else
 */ 
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

/** 
 * Manual undo elevator
 */ 
void SuperstructureController::UndoElevator(){
    robot_->SetElevatorOutput(-elevatorSlowPower_);
    robot_->SetElevatorFeederOutput(-elevatorSlowFeederPower_);
    robot_->SetIndexFunnelOutput(-indexFunnelSlowPower_);
    robot_->SetFlywheelOutput(-elevatorSlowPower_);
}

/**
 * <anual run everything: funnel, feeder, elevator
 */
void SuperstructureController::ManualFunnelFeederElevator(){
    robot_->SetElevatorOutput(elevatorSlowPower_);
    robot_->SetElevatorFeederOutput(elevatorFeederPower_);
    robot_->SetIndexFunnelOutput(indexFunnelPower_);
}

/**
 * Indexing
 */ 
void SuperstructureController::IndexUpdate(){

    // with three sensors, 5 ball indexing!

    if (funnelSensor_){
        isBallIncoming_ = true;
    }

    // figure out the state:
    
    if(currIndexLogicState_ == kReady || currIndexLogicState_ == kIdle || currIndexLogicState_ == kFull){
        if (topSensor_ && !bottomSensor_){
            // something at top and nothing at bottom, we want to move downwards: start re-indexing
            currIndexLogicState_ = kReIndexing;
            startReIndexTime_ = currTime_; // start reIndex timing (to check timeout)
        } else if (!topSensor_ && bottomSensor_){ 
            // nothing at the top, something at the bottom, ready to index upwards
            currIndexLogicState_ = kReady;
        } else if (topSensor_ && bottomSensor_){ 
            // something at the top, something at the bottom, don't move!
            currIndexLogicState_ = kFull;
        } 
        else if (!topSensor_ && !bottomSensor_){
            currIndexLogicState_ = kIdle;
        }
        else {
            nextIndexLogicState_ = kIdle;
        }
    }

    // funnel
    if (currIndexLogicState_ == kReIndexing && funnelSensor_) {
        // don't move if reIndexing and something is at the funnel
        robot_->SetIndexFunnelOutput(0.0);

    } else if (currIndexLogicState_ == kFull && funnelSensor_) {
        // don't run if full elevator and the ball has been moved to the funnel sensor
        robot_->SetIndexFunnelOutput(0.0);

    } else if (currIndexLogicState_ == kIndexingUp){
        // if indexing upwards, run funnel
        robot_->SetIndexFunnelOutput(indexFunnelPower_);
        
    } else if ((currIndexLogicState_ == kReady || currIndexLogicState_ == kIdle) 
                && funnelSensor_){ 
        // if something is in the funnel and elevator is prepped to index up
        // run index and start indexing proocess
        robot_->SetIndexFunnelOutput(0.0);
        // move on to kIndexingUp state
        currIndexLogicState_ = kIndexingUp;
        startIndexingTime_ = currTime_;

    } else if (currHandlingState_ == kIntaking){
        // otherwise, as long as the button is pressed (so kFull, kReady, kReIndexing && !funnelSensor_, kReady && !funnelSensor_)
        robot_->SetIndexFunnelOutput(indexFunnelPower_);

    } else {
        // stop.
        robot_->SetIndexFunnelOutput(0.0);

    }

    // kFull, don't move elevator
    if (currIndexLogicState_ == kFull || currIndexLogicState_ == kReady || currIndexLogicState_ == kIdle){
        robot_->SetElevatorOutput(0.0);
        robot_->SetElevatorFeederOutput(0.0);
        if (currIndexLogicState_ == kFull) {
            nextIndexLogicState_ = kFull;
        } else if (currIndexLogicState_ == kReady) {
            nextIndexLogicState_ = kReady;
        } else {
            nextIndexLogicState_ = kIdle;
        }
    }

    // kReIndexing
    if (currIndexLogicState_ == kReIndexing){
        if (bottomSensor_ || currTime_-startReIndexTime_ > 3.0){
            // if bottom or timout, STOP!
            robot_->SetElevatorOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            nextIndexLogicState_ = kReady;
        } else {
            // otherwise, move downwards
            robot_->SetElevatorOutput(-elevatorFastPower_);
            robot_->SetElevatorFeederOutput(0.0);
            nextIndexLogicState_ = kReIndexing;
        }
    }

    // kIndexingUp
    if (currIndexLogicState_ == kIndexingUp){
        if (!topSensor_ && (currTime_-startIndexingTime_ < 0.35 || bottomSensor_)){
            // as long as !top, not timeout, and something is at the bottom, move up!
            robot_->SetElevatorOutput(elevatorSlowPower_);
            robot_->SetElevatorFeederOutput(elevatorFeederPower_);
            nextIndexLogicState_ = kIndexingUp;
        } else {
            // something at the top, stop everything
            robot_->SetElevatorOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            nextIndexLogicState_ = kReIndexing;
        }
    }

    currIndexLogicState_ = nextIndexLogicState_;

}

/** 
 * Returns true if shooting is done
 * @return shoootingIsDone_, a boolean
 */ 
bool SuperstructureController::GetShootingIsDone(){
    return shootingIsDone_;
}

/** 
 * Sets shooting state
 * @param autoVelocity a double
 */ 
void SuperstructureController::SetShootingState(double autoVelocity){
    desiredFlywheelVelocity_=autoVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
    startElevatorTime_ = currTime_;
    startIndexTime_ = currTime_; 
    nextWristState_ = kRaising; // resetting whatever intake did
    nextHandlingState_ = kShooting;
}

/** 
 * Sets indexing state
 */ 
void SuperstructureController::SetIndexingState(){
    nextWristState_ = kRaising; // resetting whatever intake did
    nextHandlingState_ = kIndexing;
    if (!closePrepping_ && !farPrepping_){
        robot_->SetControlModeVelocity(0.0);
    }
}

/** 
 * Sets intaking state
 */ 
void SuperstructureController::SetIntakingState(){
    nextWristState_ = kLowering;
    nextHandlingState_ = kIntaking;
    if (!closePrepping_ && !farPrepping_){
        robot_->SetControlModeVelocity(0.0);
    }
}

/** 
 * Sets prepping state
 * @param desiredVelocity a double
 */ 
void SuperstructureController::SetPreppingState(double desiredVelocity){ // starts warming up shooter B)
    nextWristState_ = kRaising; // resetting whatever intake did
    if (!farPrepping_){ 
        shootPrepStartTime_ = robot_->GetTime();
    }
    farPrepping_ = true;
    closePrepping_ = false;
    desiredFlywheelVelocity_ = desiredVelocity;
    SetFlywheelPowerDesired(desiredFlywheelVelocity_);
}

/** 
 * Updates flywheel PID
 */ 
void SuperstructureController::FlywheelPIDControllerUpdate() {
    flywheelFFac_ = RatioFlywheel();
    robot_->ConfigFlywheelPID(flywheelPFac_, flywheelIFac_, flywheelDFac_);
}

/** 
 * Returns flywheel voltage
 * @return a double, flywheel voltage 
 */ 
double SuperstructureController::RatioFlywheel(){
    return MAX_FALCON_RPM*robot_->GetVoltage()/RATIO_BATTERY_VOLTAGE;
}

/**
 * Calculates desired flywheel velocity, uses inches
 * @return desired flywheel velocity as a double
 */ 
double SuperstructureController::CalculateFlywheelVelocityDesired() {
    double shotDistance = sqrt(pow(robot_->GetDistance()*12.0, 2.0) - pow(60.0, 2.0)) + 6.0; // all in inches
    // nasa is -320
    // into shot but slightly too high is -225
    // match 43 is -300
    double desiredVelocity = 5.58494*shotDistance + 2966.29 - 225.0 + flywheelRPMconst_; // -315 for long shots
    desiredVelocity = desiredVelocity*(flywheelPercentAdjustment_/100.0); // 92.2699% of original power
    return desiredVelocity;
}

/**
 * Sets desired flywheel power
 * @param flywheelVelocityRPM a double
 */ 
void SuperstructureController::SetFlywheelPowerDesired(double flywheelVelocityRPM) {
    double adjustedVelocity = flywheelVelocityRPM/FALCON_TO_RPM;
    double maxVelocity = RatioFlywheel()/FALCON_TO_RPM;
    double adjustedValue = (adjustedVelocity/maxVelocity*1023/adjustedVelocity);
    flywheelFFac_ = adjustedValue;
    robot_->ConfigFlywheelF(flywheelFFac_);
    
    robot_->SetControlModeVelocity(flywheelVelocityRPM/FALCON_TO_RPM);
}

/**
 * Checks if flywheel is at speed
 * @param rpm a double
 * @return true if flywheel is at speed
 */ 
bool SuperstructureController::IsFlywheelAtSpeed(double rpm){
    printf("CURRENT SPEED IS %f\n", robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM);
    double tolerance = rpm * 0.02; //tolerance at 2%

    double highTolerance = rpm*highFlywheelTolerance_;

    if (robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM > rpm-tolerance){
        numTimeAtSpeed_++;
        if (numTimeAtSpeed_ >= 5){
            atTargetSpeed_ = true;
            printf("REACHED SPEED\n");
        } else {
            atTargetSpeed_ = false;
            printf("at target speed, but hasn't been 5 seconds\n");
        }
    } else {
        numTimeAtSpeed_ = 0;
        atTargetSpeed_ = false;
    }
    return atTargetSpeed_;
}

/**
 * Returns true if is prepping for shooting
 * @return true if prepping (far or close)
 */ 
bool SuperstructureController::GetIsPrepping(){
    return (farPrepping_ || closePrepping_);
}

/**
 * Returns color of the control panel
 * @return color of panel as an std::string
 */ 
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

/**
 * Refresh shuffleboard values
 */ 
void SuperstructureController::RefreshShuffleboard(){
    flywheelRPMconst_ = flywheelRPMconstEntry_.GetDouble(0.0);
    highFlywheelTolerance_ = highFlywheelToleranceEntry_.GetDouble(0.06);
    flywheelPercentAdjustment_ = flywheelPercentAdjustmentEntry_.GetDouble(100.0);

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
    funnelLightSensorEntry_.SetBoolean(robot_->GetFunnelLightSensorStatus());
    targetSpeedEntry_.SetBoolean(atTargetSpeed_);

    funnelLeftMotorEntry_.SetDouble(robot_->GetLeftFunnelMotorStatus());
    funnelRightMotorEntry_.SetDouble(robot_->GetRightFunnelMotorStatus());
    feederMotorEntry_.SetDouble(robot_->GetFeederMotorStatus());

    jammedTimeout_ = jammedTimeoutEntry_.GetDouble(0.6);
    motorCurrentLimit_ = currentLimitEntry_.GetDouble(25.0);


//#define SHUFFLEBOARDCONTROLS
#ifdef SHUFFLEBOARDCONTROLS
    flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM); //rpm
    flywheelVelocityErrorEntry_.SetDouble(desiredFlywheelVelocity_-robot_->GetFlywheelMotor1Velocity()*FALCON_TO_RPM);
    flywheelMotor1OutputEntry_.SetDouble(robot_->FlywheelMotor1Output());
    flywheelMotor2OutputEntry_.SetDouble(robot_->FlywheelMotor2Output());
    flywheelMotor1CurrentEntry_.SetDouble(robot_->GetFlywheelMotor1Current());
    flywheelMotor2CurrentEntry_.SetDouble(robot_->GetFlywheelMotor2Current());
#endif

	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {
    flywheelRPMconstEntry_.Delete();
    highFlywheelToleranceEntry_.Delete();
    intakeWristAngleEntry_.Delete();
    flywheelPercentAdjustmentEntry_.Delete();
    
    flywheelPEntry_.Delete();
    flywheelIEntry_.Delete();
    flywheelDEntry_.Delete();
    flywheelFEntry_.Delete();


#ifdef SHUFFLEBOARDCONTROLS
    flywheelVelocityErrorEntry_.Delete();
    flywheelVelocityEntry_.Delete();
    flywheelMotor1OutputEntry_.Delete();
    flywheelMotor2OutputEntry_.Delete();
    flywheelMotor1CurrentEntry_.Delete();
    flywheelMotor2CurrentEntry_.Delete();
#endif

    elevatorBottomLightSensorEntry_.Delete();
    elevatorTopLightSensorEntry_.Delete();
    funnelLightSensorEntry_.Delete();

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

    funnelRightMotorEntry_.Delete();
    funnelLeftMotorEntry_.Delete();
    feederMotorEntry_.Delete();

    jammedTimeoutEntry_.Delete();
    currentLimitEntry_.Delete();
}