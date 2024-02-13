package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.frontDistanceState;
import org.firstinspires.ftc.teamcode.Commands.stackState;

public class distanceSensor
{
    DistanceSensor frontDistanceSensor;
    double tooFarTresh, FarTresh, CloseTresh;
    double distMulti;
    double offset;
    frontDistanceState frontDistanceStates;
    double timer = 0;

    public distanceSensor(HardwareMap hardwareMap)
    {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
    }

    public frontDistanceState frontAdjust()
    {
        frontDistanceStates = frontDistanceState.noDetect;

        if(frontDistanceSensor.getDistance(DistanceUnit.INCH) > 4)
        {
            frontDistanceStates = frontDistanceState.Far;
        }
        return frontDistanceStates;
    }
}
