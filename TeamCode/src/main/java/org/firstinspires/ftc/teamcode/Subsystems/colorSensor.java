package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class colorSensor {
     ColorSensor leftColorSensor, rightColorSensor;
     double leftRedVal, leftGreenVal, leftBlueVal;
     double alphaValue; //light intensity
     double targetValue = 1000;  //might have to change
    boolean stackFound = false;

    public colorSensor(HardwareMap hardwareMap){
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor") ;
        rightColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor") ;
    }

//    public double getRedValue()
//    {
//        return ColorSensorr.red();
//    }
//    public double getBlueValue()
//    {
//        return ColorSensorr.blue();
//    }
//    public double getGreenValue()
//    {
//        return ColorSensorr.green();
//    }
//    public double getAlphaValue(){
//        return ColorSensorr.alpha();
//    }

    public double align()
    {
//        leftRedVal = leftColorSensor.red();
//        leftGreenVal = leftColorSensor.green();
//        leftBlueVal = leftColorSensor.blue();
//
//        rightRedVal = rightColorSensor.red();
//        rightGreenVal = rightColorSensor.green();
//        rightBlueVal = rightColorSensor.blue();

        if(((leftRedVal == 255 && leftGreenVal == 255 && leftBlueVal == 255 )
    }
}
