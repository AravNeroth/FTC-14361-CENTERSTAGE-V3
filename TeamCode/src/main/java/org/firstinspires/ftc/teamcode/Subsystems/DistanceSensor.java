package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceSensor
{
    com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;


    double directionOffset = -1;

    int stackYPose = 36;
    int stackXPose = 48;
    int robotFrontBackOffset = 9;
    int robotSideOffset = 8;


    public DistanceSensor(HardwareMap hardwareMap){
        // if you don't enter an alliance, then it defaults to lid distance sensor
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "frontDist");
    }

    public double getFrontStackDistance(){
        double xOffset = 0;
        double currentFrontdistance = - (72 - distanceSensor.getDistance(DistanceUnit.INCH)) + robotFrontBackOffset;

        if(currentFrontdistance < stackXPose) {
            xOffset = -(currentFrontdistance + stackXPose);
            return xOffset;
        }

        else if(currentFrontdistance > stackXPose) {
            xOffset = stackXPose - currentFrontdistance;
            return xOffset;
        }

        return xOffset;
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

}