package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.frontDistanceState;
import org.firstinspires.ftc.teamcode.Commands.stackState;

public class distanceSensor
{
    DistanceSensor frontDistanceSensor, leftDistanceSensor, rightDistanceSensor;

    double tooFarTresh, FarTresh, CloseTresh;
    double distMulti;
    double offset;
    frontDistanceState frontDistanceStates;
    double timer = 0;

    public distanceSensor(HardwareMap hardwareMap)
    {
       // frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

//    public frontDistanceState frontAdjust()
//    {
//        frontDistanceStates = frontDistanceState.noDetect;
//
//        if(frontDistanceSensor.getDistance(DistanceUnit.INCH) > 4)
//        {
//            frontDistanceStates = frontDistanceState.Far;
//        }
//        return frontDistanceStates;
//    }
    public double getLeftDistance(){
        return leftDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getLeftDistanceEdgeDistance(){
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) - 4;
    }
      public double getBotsLeftCenterDistance(){
    return leftDistanceSensor.getDistance(DistanceUnit.INCH) + 3.75;
}
    public double getRightDistance(){
        return rightDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getRightEdgeDistance(){
        return rightDistanceSensor.getDistance(DistanceUnit.INCH) - 3;
    }
    public double getBotsRightCenterDistance(){
        return rightDistanceSensor.getDistance(DistanceUnit.INCH) + 4.75;
    }
}
