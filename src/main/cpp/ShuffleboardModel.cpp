/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"

void RobotModel::CreateDriveStraightPID(){

    //anglePIDLayout_(GetPIDTab().GetLayout("Angle", "List Layout"));
	//distancePIDLayout_(GetPIDTab().GetLayout("Distance", "List Layout"));

    aPEntry_ = anglePIDLayout_.Add("P", 0.08).GetEntry();
    aIEntry_ = anglePIDLayout_.Add("I", 0.0).GetEntry();
    aDEntry_ = anglePIDLayout_.Add("D", 0.02).GetEntry();
    dPEntry_ = distancePIDLayout_.Add("P", 0.8).GetEntry();
    dIEntry_ = distancePIDLayout_.Add("I", 0.0).GetEntry();
    dDEntry_ = distancePIDLayout_.Add("D", 0.2).GetEntry();
}

void RobotModel::CreatePivotPID() {
    pEntry_ = pivotPIDLayout_.Add("P", 0.031).GetEntry();
    iEntry_ = pivotPIDLayout_.Add("I", 0.0).GetEntry();
    dEntry_ = pivotPIDLayout_.Add("D", 0.017).GetEntry();
}

double RobotModel::GetDriveStraightAngleP() {
    return aPEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightAngleI() {
    return aIEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightAngleD() {
    return aDEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightDriveP() {
    return dPEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightDriveI() {
    return dIEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightDriveD() {
    return dDEntry_.GetDouble(0.0);
}

double RobotModel::GetPivotP() {
    return pEntry_.GetDouble(0.0);
}

double RobotModel::GetPivotI() {
    return iEntry_.GetDouble(0.0);
}

double RobotModel::GetPivotD() {
    return dEntry_.GetDouble(0.0);
}

