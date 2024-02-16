package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class colorSensor {
     ColorSensor ColorSensorr;
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
    public String getColor(){

        if((ColorSensorr.red() > 250 && ColorSensorr.red() < 350)&& (ColorSensorr.blue() > 100 && ColorSensorr.blue() < 200) && (ColorSensorr.green() > 450 && ColorSensorr.green() <550)){
            return "Yellow";
        } else if ((ColorSensorr.red() > 80 && ColorSensorr.red() < 180) && (ColorSensorr.blue() > 150 && ColorSensorr.blue() < 250) && (ColorSensorr.green() > 315 && ColorSensorr.green() <415)  ) {
            return "Green";
        }
        else if((ColorSensorr.red() > 225 && ColorSensorr.red() < 325 ) && (ColorSensorr.blue() > 475 && ColorSensorr.blue() < 575 )&& (ColorSensorr.green() > 365 && ColorSensorr.green() <465) ){
            return "Purple";
        }
        else if((ColorSensorr.red() > 450 && ColorSensorr.red() < 550 ) && (ColorSensorr.blue() > 800 && ColorSensorr.blue() < 900 )&& (ColorSensorr.green() > 900 && ColorSensorr.green() <1000)){
            return "White";
        }
        else{
           return "Pixel Not Detected";
        }
    }

}
