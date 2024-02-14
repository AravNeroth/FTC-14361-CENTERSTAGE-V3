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

        if((ColorSensorr.red() > 350 && ColorSensorr.red() < 750)&& (ColorSensorr.blue() > 300 && ColorSensorr.blue() < 700) && (ColorSensorr.green() > 500 && ColorSensorr.green() <900)){
            return "Yellow";
        } else if ((ColorSensorr.red() > 200 && ColorSensorr.red() < 400) && (ColorSensorr.blue() > 250 && ColorSensorr.blue() < 550) && (ColorSensorr.green() > 500 && ColorSensorr.green() <700)  ) {
            return "Green";
        }
        else if((ColorSensorr.red() > 300 && ColorSensorr.red() < 600 ) && (ColorSensorr.blue() > 600 && ColorSensorr.blue() < 1000 )&& (ColorSensorr.green() > 530 && ColorSensorr.green() <930) ){
            return "Purple";
        }
        else if((ColorSensorr.red() > 800 && ColorSensorr.red() < 1200 ) && (ColorSensorr.blue() > 1400 && ColorSensorr.blue() < 1800 )&& (ColorSensorr.green() > 1200 && ColorSensorr.green() <1600)){
            return "White";
        }
        else{
           return "Pixel Not Detected";
        }
    }
    public String proportionalColorDetect(){
        int green = ColorSensorr.green()/100;
        int red = ColorSensorr.red()/100;
        int blue = ColorSensorr.green()/100;
        if(red/3 ==2 && green/3 ==3 && blue/4 ==3){
            return "Purple";
        }
        else return "White";
    }

}
