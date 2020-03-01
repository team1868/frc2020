/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"

void RobotModel::CreatePIDEntries(){
    //last tuned: 2/20 without climber2
    aPEntry_ = anglePIDLayout_.Add("P", 0.0055).GetEntry(); // 0.0055 for practice bot
    aIEntry_ = anglePIDLayout_.Add("I", 0.0).GetEntry();
    aDEntry_ = anglePIDLayout_.Add("D", 0.01).GetEntry();
    dPEntry_ = distancePIDLayout_.Add("P", 0.125).GetEntry(); // 0.19 for practice bot
    dIEntry_ = distancePIDLayout_.Add("I", 0.0).GetEntry();
    dDEntry_ = distancePIDLayout_.Add("D", 0.11).GetEntry(); // 0.08 for practice bot

    // pEntry_ = pivotPIDLayout_.Add("P", 0.031).GetEntry();
    // iEntry_ = pivotPIDLayout_.Add("I", 0.0).GetEntry();
    // dEntry_ = pivotPIDLayout_.Add("D", 0.017).GetEntry();
    pEntry_ = pivotPIDLayout_.Add("P", 0.016).GetEntry(); // 0.03225 for nova, 0.0247 for practice bot 0.016 for comp bot
    iEntry_ = pivotPIDLayout_.Add("I", 0.0).GetEntry();
    dEntry_ = pivotPIDLayout_.Add("D", 0.0115).GetEntry(); // 0.0173 for nova, 0.0162 for practice bot 0.0115 for comp bot

    dPFacNet_ =  curveDistancePIDLayout_.Add("Curve dP", 0.1).GetEntry();
    dIFacNet_ =  curveDistancePIDLayout_.Add("Curve dI", 0.0).GetEntry();
    dDFacNet_ =  curveDistancePIDLayout_.Add("Curve dD", 0.0).GetEntry();
    //tPFacNet_ =  curveTurnPIDLayout_.Add("Curve tP", 0.07).GetEntry();
    //tIFacNet_ =  curveTurnPIDLayout_.Add("Curve tI", 0.0).GetEntry();
    //tDFacNet_ =  curveTurnPIDLayout_.Add("Curve tD", 0.0).GetEntry();

    pEntryP_ = pointPIDLayout_.Add("P", 0.0).GetEntry();
    iEntryP_ = pointPIDLayout_.Add("I", 0.0).GetEntry();
    dEntryP_ = pointPIDLayout_.Add("D", 0.0).GetEntry();
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

double RobotModel::GetDriveStraightDistanceP() {
    return dPEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightDistanceI() {
    return dIEntry_.GetDouble(0.0);
}

double RobotModel::GetDriveStraightDistanceD() {
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

double RobotModel::GetCurveDistanceP(){
    return dPFacNet_.GetDouble(0.0);
}
double RobotModel::GetCurveDistanceI(){
    return dIFacNet_.GetDouble(0.0);
}
double RobotModel::GetCurveDistanceD(){
    return dPFacNet_.GetDouble(0.0);
}
/*
double RobotModel::GetCurveTurnP(){
    return tPFacNet_.GetDouble(0.0);
}
double RobotModel::GetCurveTurnI(){
    return tIFacNet_.GetDouble(0.0);
}
double RobotModel::GetCurveTurnD(){
    return tDFacNet_.GetDouble(0.0);
}
*/

double RobotModel::GetPointP() {
    return pEntryP_.GetDouble(0.0);
}

double RobotModel::GetPointI() {
    return iEntryP_.GetDouble(0.0);
}

double RobotModel::GetPointD() {
    return dEntryP_.GetDouble(0.0);
}


