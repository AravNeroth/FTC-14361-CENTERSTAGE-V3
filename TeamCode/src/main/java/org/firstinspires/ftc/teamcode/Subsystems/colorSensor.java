package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.stackState;

public class colorSensor
{
    ColorSensor leftColorSensor, rightColorSensor;
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

    public colorSensor(HardwareMap hardwareMap)
    {
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");

        offset = 2;
    }

    public stackState align(int numOfAttempts)
    {
        timer = numOfAttempts;
        stackStates = stackState.noDetect;

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

    public int getLeftRedVal()
    {
        return leftColorSensor.red();
    }

    public int getLeftGreenVal()
    {
        return leftColorSensor.green();
    }

    public int getLeftBlueVal()
    {
        return leftColorSensor.blue();
    }

    public int getRightRedVal()
    {
        return rightColorSensor.red();
    }

    public int getRightGreenVal()
    {
        return rightColorSensor.green();
    }

    public int getRightBlueVal()
    {
        return rightColorSensor.blue();
    }
}
