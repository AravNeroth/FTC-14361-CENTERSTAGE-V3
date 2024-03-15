package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.holderServoState;
import org.firstinspires.ftc.teamcode.util.robotConstants;

//port is

public class holderServo
{
    private ServoEx holderServo;
    double minAngle = 0, maxAngle= 360;
    public holderServo(HardwareMap hardwareMap)
    {
        holderServo = new SimpleServo(hardwareMap, "bucketServo", minAngle, maxAngle, AngleUnit.DEGREES);
    }

    public void setHolderServoPosition(holderServoState holderServoState)
    {
        switch(holderServoState)
        {
            case open:
                holderServo.setPosition(robotConstants.holderServo.open);
                break;
            case close:
                holderServo.setPosition(robotConstants.holderServo.close);
                break;
            default:
                holderServo.setPosition(robotConstants.holderServo.open);
        }
    }
    public double getHolderServoPosition()
    {
        return holderServo.getPosition();
    }
    public void setHolderServoCustomPosition(double position){
        holderServo.setPosition(position);
    }

}