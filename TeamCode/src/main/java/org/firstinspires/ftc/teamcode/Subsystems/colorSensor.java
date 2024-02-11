package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class colorSensor {
     ColorSensor ColorSensorr;
   //  String purple = "purple", green = "green", red = "red", white = "white";
     double redValue, greenValue, blueValue;
     double alphaValue; //light intensity
     double targetValue = 1000;  //might have to change
    public colorSensor(HardwareMap hardwareMap){
        ColorSensorr = hardwareMap.get(ColorSensor.class, "colorSensor") ;
    }
    public double getRedValue(){
        return ColorSensorr.red();
    }
    public double getBlueValue(){
        return ColorSensorr.blue();
    }
    public double getGreenValue(){
        return ColorSensorr.green();
    }
    public double getAlphaValue(){
        return ColorSensorr.alpha();
    }
//    public String getColor(){
//        if(getRedValue() >
//    }

}
