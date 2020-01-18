#!/bin/sh

gcc -c -I/mnt/c/mopro_text/ ./pathfinder/*.c
gcc -c -I/mnt/c/mopro_text/ ./pathfinder/*/*.c
g++ -c -I/mnt/c/mopro_text/ ./CalcPts.cpp
g++ -L CalcPts.lib error.o generator.o io.o mathutil.o spline.o trajectory.o distance.o hermite.o swerve.o encoder.o tank.o CalcPts.o -o MpTest1
