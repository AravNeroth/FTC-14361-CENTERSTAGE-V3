package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StackColorSensors {
    public ColorSensor leftColor, rightColor;

    private Boolean lColor, rColor;

    public StackColorSensors(HardwareMap hardwareMap) {
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        lColor = false;
        rColor = false;
    }

    public void scanLeftColor(){
        if((leftColor.red() > 800 && leftColor.red() < 1200 ) && (leftColor.blue() > 1400 && leftColor.blue() < 1800 ) && (leftColor.green() > 1200 && leftColor.green() < 1600))
            lColor = true;
    }

    public void scanRightColor(){
        if((rightColor.red() > 800 && rightColor.red() < 1200 ) && (rightColor.blue() > 1400 && rightColor.blue() < 1800 ) && (rightColor.green() > 1200 && rightColor.green() < 1600))
            rColor = true;
    }
    public Boolean getlColor(){
        scanLeftColor();
        return lColor;
    }

    public Boolean getrColor(){
        scanRightColor();
        return rColor;
    }

}
