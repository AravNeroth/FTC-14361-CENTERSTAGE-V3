package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.stackState;

public class ColorSensor
{
    com.qualcomm.robotcore.hardware.ColorSensor leftColorSensor, rightColorSensor;
    double leftRedVal, leftGreenVal, leftBlueVal;
    double rightRedVal, rightGreenVal, rightBlueVal;
    double redTresh, greenTresh, blueTresh;
    double alphaValue;
     //light intensity
    double distMulti;
    double offset;
    boolean midAlignment = false;
    stackState stackStates;
    double timer = 0;

    public ColorSensor(HardwareMap hardwareMap)
    {
        leftColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");

        offset = 2;
    }

    public stackState align(int numOfAttempts)
    {
        timer = numOfAttempts;
        distMulti = .5;
        stackStates = stackState.noDetect;

        if(numOfAttempts < 3)
        {
            offset = offset * .5;
        }

        if((leftRedVal > redTresh && leftGreenVal > greenTresh && leftBlueVal > blueTresh) && (rightRedVal > redTresh && rightGreenVal > greenTresh && rightBlueVal > blueTresh))
        {
            stackStates = stackState.midAligned;
        }
        else if((leftRedVal > redTresh && leftGreenVal > greenTresh && leftBlueVal > blueTresh) && (rightRedVal < redTresh && rightGreenVal < greenTresh && rightBlueVal < blueTresh))
        {
            stackStates = stackState.offLeft;
        }
        else if((leftRedVal < redTresh && leftGreenVal < greenTresh && leftBlueVal < blueTresh) && (rightRedVal < redTresh && rightGreenVal < greenTresh && rightBlueVal < blueTresh))
        {
            stackStates = stackState.offRight;
        }
        return stackStates;
    }

    public double getOffset()
    {
        return offset;
    }
}
