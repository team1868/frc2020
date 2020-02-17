/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
#include <math.h>
using namespace std;

SuperstructureController::SuperstructureController(RobotModel *robot, ControlBoard *humanControl) :
    flywheelPIDLayout_(robot->GetSuperstructureTab().GetLayout("Flywheel", "List Layout").WithPosition(3, 3)),
    sensorsLayout_(robot->GetSuperstructureTab().GetLayout("Sensors", "List Layout").WithPosition(0, 1)),
    manualOverrideLayout_(robot->GetModeTab().GetLayout("climboverride", "List Layout").WithPosition(1,1))
    {
    
    robot_ = robot;
    humanControl_ = humanControl; 

    // fix all of this
    climbElevatorUpPower_ = 0.5; // fix
    climbElevatorDownPower_ = -0.4; // fix
    bool positiveDirection_ = true;
    climbWinchPower_ = 0.5; // fix
    
    desiredFlywheelVelocity_ = 1.0;//7000; // ticks per 0.1 seconds
    closeFlywheelVelocity_ = 1.0;//5000.0*2048.0/60000.0;//2000; //5000 r/m * 2048 tick/r / (60 s/m * 1000 ms/s)
    flywheelResetTime_ = 2.0; // fix //why does this exist
    // create talon pid controller
    // fix encoder source
    std::cout << "start flywheel encoder creation" << std::endl << std::flush;
    //flywheelEncoder1_ = &robot_->GetFlywheelMotor1()->GetSensorCollection();
    //flywheelEncoder2_ = &robot_->GetFlywheelMotor2()->GetSensorCollection();
    std::cout << "end flywheel encoder creation" << std::endl << std::flush;
    
    // create talon pid controller
    
    flywheelPIDSource_ = new TalonFXPIDSource(robot_);
    flywheelPIDOutput_ = new SuperstructurePIDOutput();
    flywheelPID_ = new PIDController(flywheelPFac_, flywheelIFac_, flywheelDFac_, flywheelFFac_, flywheelPIDSource_, flywheelPIDOutput_);
    

    elevatorFeederPower_ = 1.0; // fix
    elevatorSlowPower_ = 0.5; //fix
    elevatorFastPower_ = 1.0; //fix
    indexFunnelPower_ = 0.0; // fix
    lowerElevatorTimeout_ = 4.0; //fix
    elevatorTimeout_ = 4.0;
    //lastBottomStatus_ = false;

    wristPFac_ = 0.03;

    controlPanelPower_ = 0.5; // fix
    controlPanelCounter_ = 0;

    desiredIntakeWristAngle_ = 110.0; // fix later :) probably wrong, ask mech

    closePrepping_ = false;
    farPrepping_ = false;

    currTime_ = robot_->GetTime();
    shootPrepStartTime_ = currTime_; //TODO not used rn
    startResetTime_ = currTime_;
    resetTimeout_ = 2.0;

    currState_ = kIndexing;
	nextState_ = kIndexing;
    currAutoState_ = kInit;
    nextAutoState_ = kAutoIndexing;
    currIndexState_ = kIndexInit;
    nextIndexState_ = kIndexInit;
    currWristState_ = kRaising;
    
    currTime_ = robot_->GetTime();
    startElevatorTime_ = currTime_;
    startIndexTime_ = currTime_;
    startResetTime_ = currTime_;

    // shuffleboard
    flywheelVelocityEntry_ = flywheelPIDLayout_.Add("flywheel velocity", 0.0).GetEntry();
    flywheelVelocityErrorEntry_ = flywheelPIDLayout_.Add("flywheel error", 0.0).GetEntry();
    
    flywheelPEntry_ = flywheelPIDLayout_.Add("flywheel P", 0.0).GetEntry();
    flywheelIEntry_ = flywheelPIDLayout_.Add("flywheel I", 0.0).GetEntry();
    flywheelDEntry_ = flywheelPIDLayout_.Add("flywheel D", 0.0).GetEntry();
    flywheelFFEntry_ = flywheelPIDLayout_.Add("flywheel FF", 0.0).GetEntry();

    wristPEntry_ = robot_->GetSuperstructureTab().Add("wrist P", 0.0).GetEntry();

    winchAutoEntry_ = manualOverrideLayout_.Add("override climber", 0.0).GetEntry();

    elevatorTopLightSensorEntry_ = sensorsLayout_.Add("bottom elevator", false).GetEntry();
    elevatorBottomLightSensorEntry_ = sensorsLayout_.Add("top elevator", false).GetEntry();
    intakeWristAngleEntry_ = sensorsLayout_.Add("intake wrist angle", 0.0).GetEntry();
    printf("end of superstructure controller constructor\n");
}


void SuperstructureController::Reset() { // might not need this
    currState_ = kIndexing;
	nextState_ = kIndexing;
    // check whether autostates should be reset here or should 
    currAutoState_ = kAutoInit;
    nextAutoState_ = kAutoIndexing;
    currWristState_ = kRaising;
}


void SuperstructureController::WristUpdate(){

    currWristAngle_ = robot_->GetIntakeWristAngle(); // might not need?
    switch (currWristState_){
        case kRaising:
            // might not need lowering if we have an idle
            robot_->SetIntakeRollersOutput(0.0);
            if(currWristAngle_ > 0.0) {
                robot_->SetIntakeWristOutput((0.0-currWristAngle_)*wristPFac_); // check if potentiometer returns angle
            }
            else{
                robot_->SetIntakeWristOutput(0.0);
            }
            break;
        case kLowering:
            printf("in WRIST kLowering\n");
            if(currWristAngle_ < desiredIntakeWristAngle_) {
                robot_->SetIntakeWristOutput((desiredIntakeWristAngle_-currWristAngle_)*wristPFac_);
            }
            else{
                robot_->SetIntakeWristOutput(0.0);
            }
            if(currWristAngle_ > desiredIntakeWristAngle_ - 20.0){ //within acceptable range
                robot_->SetIntakeRollersOutput(CalculateIntakeRollersPower());
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
            desiredFlywheelVelocity_ = closeFlywheelVelocity_;
            SetFlywheelPowerDesired(desiredFlywheelVelocity_);
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
            desiredFlywheelVelocity_ = CalculateFlywheelVelocityDesired();
            SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION
            robot_->EngageFlywheelHood(); // hood implemented
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
    //winch: automatic or human 


    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbWinchLeftButton)){
        robot_->SetClimbWinchLeftOutput(climbWinchPower_);
    } else {
        robot_->SetClimbWinchLeftOutput(0.0);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbWinchRightButton)){
        robot_->SetClimbWinchRightOutput(climbWinchPower_);
    }else {
        robot_->SetClimbWinchRightOutput(0.0);
    }

    //human override state

    if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
        currState_ = kControlPanelStage2; // verify
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kControlPanelStage2Button)){
        currState_ = kControlPanelStage3;
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbElevatorUpButton)){
        positiveDirection_ = true;
        currState_ = kClimbingElevator;
    } else if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbElevatorDownButton)){
        positiveDirection_ = false;
        currState_ = kClimbingElevator;
    } else {
        robot_->SetClimberElevatorOutput(0.0);
    }

    // independent climbing buttons, please move setting output from main
    /*
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbWinchLeftButton)){
        robot_->SetClimbWinchLeftOutput(climbWinchPower_);
    } else {
        robot_->SetClimbWinchLeftOutput(0.0);
    }
    if(humanControl_->GetDesired(ControlBoard::Buttons::kClimbWinchRightButton)){
        robot_->SetClimbWinchRightOutput(climbWinchPower_);
    }else {
        robot_->SetClimbWinchRightOutput(0.0);
    }*/

    if(humanControl_->GetDesired(ControlBoard::Buttons::kIntakeSeriesButton)){
        currState_ = kIntaking;
    } else if (humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton)/* && 
               currTime_ - shootPrepStartTime_ > 1.0*/){ //TODO remove this
        if(currState_!=kShooting){
            startIndexTime_ = currTime_;
        }
        currState_ = kShooting; 
    } else if(/*!humanControl_->GetDesired(ControlBoard::Buttons::kShootingButton) && */
              currState_ == kShooting){ //shooting to decide not to shoot
        currState_ = kResetting;
    } else if(currState_ != kResetting){ //not intaking, shooting, or resetting, only option is indexing or prepping (also includes indexing)
        currState_ = kIndexing;
    }
    
    //flywheel control if not shooting
    if (currState_ != kShooting){
        if(humanControl_->GetDesired(ControlBoard::Buttons::kShootClosePrepButton)){
            if(!closePrepping_){
                shootPrepStartTime_ = currTime_;
                printf("start close prep shooting\n");
            }
            printf("in close PREPPING -------------\n");
            desiredFlywheelVelocity_ = closeFlywheelVelocity_;
            robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
            //SetFlywheelPowerDesired(desiredFlywheelVelocity_);
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
            robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
            //SetFlywheelPowerDesired(desiredFlywheelVelocity_); //TODO INTEGRATE VISION
            robot_->EngageFlywheelHood(); //TODO add if distance > x
            closePrepping_ = false;
            farPrepping_ = true;
        } else {
            printf("STOPPING FLYWHEEL\n");
            robot_->SetFlywheelOutput(0.0);
            robot_->DisengageFlywheelHood();
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

    //TODO replace "//robot_->SetArm(bool a);" with if !sensorGood set arm power small in bool a direction
    //in current code: true is arm down and false is arm up
    //TODO ADD CLIMBING AND SPINNER SEMIAUTO
    switch(currState_){
        case kIndexing:
            IndexUpdate();
            printf("in kIndexing\n");

            //robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        case kIntaking:
            printf("in kIntaking\n");
            //robot_->SetIntakeRollersOutput(CalculateIntakeRollersPower());
            currWristState_ = kLowering;
            //robot_->SetArm(true); TODO IMPLEMENT
            IndexUpdate();
            break;
        case kShooting:
            printf("in kShooting with %f\n", desiredFlywheelVelocity_);
            robot_->SetFlywheelOutput(desiredFlywheelVelocity_);
            //SetFlywheelPowerDesired(desiredFlywheelVelocity_);
            //raise elevator if not at speed, OR nothing at top and not timed out at bottom
            if(IsFlywheelAtSpeed() || (!topSensor_ && !bTimeout_)){
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

            if(tTimeout_ && bTimeout_){
                nextState_ = kIndexing;//kResetting;
                //robot_->SetFlywheelOutput(0.0);
                startResetTime_ = currTime_;
            }

            //robot_->SetIntakeRollersOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        case kResetting:
            printf("in kResetting\n");
            robot_->SetFlywheelOutput(0.0);

            if(!bottomSensor_ && currTime_-startResetTime_ <= resetTimeout_){
                robot_->SetElevatorOutput(-elevatorFastPower_); //bring down elevator
            } else {
                robot_->SetElevatorOutput(0.0);
                nextState_ = kIndexing;
            }

            //robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
            break;
        default:
            printf("ERROR: no state in superstructure controller\n");
            robot_->SetFlywheelOutput(0.0);
            //robot_->SetIntakeRollersOutput(0.0);
            robot_->SetIndexFunnelOutput(0.0);
            robot_->SetElevatorFeederOutput(0.0);
            currWristState_ = kRaising;
            //robot_->SetArm(false); TODO IMPLEMENT
    }

    printf("DESIRED FLYWHEEL VELOCITY %f\n", desiredFlywheelVelocity_);
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

    
    robot_->ConfigFlywheelP(flywheelPFac_);
    robot_->ConfigFlywheelI(flywheelIFac_);
    robot_->ConfigFlywheelD(flywheelDFac_);
    robot_->ConfigFlywheelF(flywheelFFac_);
    // closed-loop error maybe?

}

double SuperstructureController::CalculateFlywheelVelocityDesired() {
    return 1.0;//2000; // fix
}

//TODO actually implement
void SuperstructureController::SetFlywheelPowerDesired(double flywheelVelocity) {
    robot_->SetControlModeVelocity(flywheelVelocity);
}

bool SuperstructureController::IsFlywheelAtSpeed(){
    //if(robot_->GetFlywheelMotor1Velocity() > desiredFlywheelVelocity_-50 && robot_->GetFlywheelMotor1Velocity() < desiredFlywheelVelocity_+50){
        return true; // random threshold value, change as desired
    //}
    //return false;
}


void SuperstructureController::WinchUpdate() {
    double currRobotAngle_ = (atan(tan(robot_-> GetNavXPitch()) * sin(robot_ -> GetNavXYaw())));
    double initRightEncoderVal = robot_->GetClimberWinchRightEncoderValue();
    double initLeftEncoderVal = robot_->GetClimberWinchRightEncoderValue();
    double ticksPerFt = 256/4.32/12.0; // ticks per rotation / circumference of drum

    if (currRobotAngle_ < 0.0){
        if(robot_->GetClimberWinchRightEncoderValue() < initRightEncoderVal + (ROBOT_WIDTH*sin(currRobotAngle_)*ticksPerFt)){
            robot_->SetClimbWinchRightOutput(climbWinchPower_);
        }
        else{
            robot_->SetClimbWinchRightOutput(0.0);
        }
    }
    else if (currRobotAngle_ > 0.0){
        if(robot_->GetClimberWinchLeftEncoderValue() < initLeftEncoderVal + (ROBOT_WIDTH*sin(currRobotAngle_)*ticksPerFt)) {
            robot_->SetClimbWinchLeftOutput(climbWinchPower_);
        }
        else{
            robot_->SetClimbWinchRightOutput(0.0);
        }
    }
    else{
        robot_->SetClimbWinchRightOutput(0);
        robot_->SetClimbWinchLeftOutput(0);
    }

}

bool SuperstructureController::IndexUpdate(){

    //printf("top sensor %f and bottom sensor %f\n", topSensor_, bottomSensor_);

    //control top
    if(!topSensor_ && bottomSensor_){
        //printf("RUNNING TOP ELEVATOR\n");
        printf("running elevator");
        robot_->SetElevatorOutput(elevatorFastPower_);
    } else {
        printf("not running elevator");
        robot_->SetElevatorOutput(0.0);
    }

    //control bottom
    if(!bottomSensor_ && (!bTimeout_ || currState_ == kIntaking)){
        //robot_->SetIndexFunnelOutput(indexFunnelPower_); //TODO PUT BACK IN
        robot_->SetElevatorFeederOutput(elevatorFeederPower_);
        printf("RUNNNNINGGGG FUNNEL AND FEEDER\n");
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

//TODO actually implement
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
    flywheelFFac_ = flywheelFFEntry_.GetDouble(0.0);
    FlywheelPIDControllerUpdate();

    wristPFac_ = wristPEntry_.GetDouble(0.03);

    //flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelEncoder1Velocity()*8*M_PI/60); (figure out what units this is generated in)
    currWristAngle_ = robot_->GetIntakeWristAngle();

    flywheelVelocityEntry_.SetDouble(robot_->GetFlywheelMotor1Velocity()/2048.0*60000.0); //rpm
    flywheelVelocityErrorEntry_.SetDouble(desiredFlywheelVelocity_-robot_->GetFlywheelMotor1Velocity());

	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();
}

SuperstructureController::~SuperstructureController() {
    intakeWristAngleEntry_.Delete();
    flywheelPEntry_.Delete();
    flywheelIEntry_.Delete();
    flywheelDEntry_.Delete();
    flywheelFFEntry_.Delete();
    winchAutoEntry_.Delete();
    flywheelVelocityErrorEntry_.Delete();
    flywheelVelocityEntry_.Delete();
    wristPEntry_.Delete();
    elevatorBottomLightSensorEntry_.Delete();
    elevatorTopLightSensorEntry_.Delete();
    if (flywheelPID_ != NULL) {
        flywheelPID_->Disable();
        delete flywheelPID_;
    } 
}
