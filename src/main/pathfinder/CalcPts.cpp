#include <iostream>
#include <cstdio>
extern "C"{
#include "pathfinder.h"
}

const static int NUM_WAYPTS = 2;
using namespace std;

int main(){
    Waypoint pts[NUM_WAYPTS];
    TrajectoryCandidate candidate;


    Waypoint p1 = {0, 0, d2r(90)};
    pts[0] = p1;
    Waypoint p2 = {1, 3, d2r(90)};
    pts[1] = p2;
    // Waypoint p3 = {-2, 8, 0};

    double timeStep = 0.02; // s, for TimedRobot
    double maxVeloc = 1.75; // m/s new ratio
    double maxAccel = 1.2; // m/s/s, UNTUNED
    double maxJerk = 0.7; // m/s/s/s

    pathfinder_prepare(pts, NUM_WAYPTS, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, timeStep, maxVeloc, maxAccel, maxJerk, &candidate);

    int len = candidate.length;
    Segment *trajectory = (Segment*)malloc(len * sizeof(Segment));

    int result = pathfinder_generate(&candidate, trajectory);
    if(result<0){
        printf("Trajectory not generated\n");
    }
    
    Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment)*len);
    Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment)*len);
    
    double wheelbaseWidth = 0.64; //meters

    pathfinder_modify_tank(trajectory, len, leftTrajectory, rightTrajectory, wheelbaseWidth); 
    
    //double curTime = timeStep;
    int i;
    //double ftInM = 3.28084;
    for (i = 0; i < len; i++) {
        //Segment s = trajectory[i];
        Segment s = leftTrajectory[i];
        Segment s2 = rightTrajectory[i];
        //time, dist, veloc, accel, jerk, x, y, angle in degrees
        //printf("%f, %f, %f, %f, %f, %f, %f, %f,\n", curTime, s.position, s.velocity, s.acceleration, s.jerk, s.x, s.y, s.heading*180.0/PI-90);
        printf("%f, %f, %f, %f, %f, %f, %f, %f,\n", s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);
        printf("%f, %f, %f, %f, %f, %f, %f, %f, \n", s2.dt, s2.x, s2.y, s2.position, s2.velocity, s2.acceleration, s2.jerk, s2.heading);  
        //curTime += timeStep;
        
	// printf("Time Step: %f\n", s.dt);
        // printf("Coords: (%f, %f)\n", s.x, s.y);
        // printf("Position (Distance): %f\n", s.position);
        // printf("Velocity: %f\n", s.velocity);
        // printf("Acceleration: %f\n", s.acceleration);
        // printf("Jerk (Acceleration per Second): %f\n", s.jerk);
        // printf("Heading (radians): %f\n", s.heading);
    }
   
    //TankModifier modifier = new TankModifier(trajectory).modify(0.5);
    //EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
    //EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
    //double wheelDiam = 4.0/12.0;
    //double encoderCountPerRev = 256.0;
    //left.configureEncoder(getEncPosition(), encoderCountPerRev, wheelDiam);
    //double pPID = 0.8, iPID = 0.0, dPID = 0.2;
    //double velocRatio = 1.0/maxVeloc;
    //double accelGain = 0; //to get to a higher/lower speed quicker
    //left.configurePIDVA(pPID, iPID, dPID, velocRatio, accelGain);
    //double lOutput = left.calculate(getEncPosition());


    free(trajectory);
    free(leftTrajectory);
    free(rightTrajectory);
    //printf("Exit\n");
    //printf("Done :)\n");

    return 0;
}

