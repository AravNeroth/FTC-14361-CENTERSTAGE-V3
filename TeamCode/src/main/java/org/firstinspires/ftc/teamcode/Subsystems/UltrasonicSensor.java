package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

//left US is 23
//right US is 01
public class UltrasonicSensor {

    private AnalogInput leftUS, rightUS;
    // private int maxR = 520;

    public UltrasonicSensor(HardwareMap hardwareMap) {
        leftUS = hardwareMap.get(AnalogInput.class, "leftUltraSonicSensor");
     //   rightUS = hardwareMap.get(AnalogInput.class, "rightUltraSonicSensor");
    }


    //this is the distance to the ultrasonic itself
    public double getVoltageWEquation(){
        double distance = 88.7*(leftUS.getVoltage())-15.2 + 3.2;
        return distance;
    }
    public double getLeftDistanceCenter(){
        double distance = 88.7*(leftUS.getVoltage())-15.2 + 8;
        return distance;
    }
    public double getLeftDistanceEdge(){
        double distance = 88.7*(leftUS.getVoltage())-15.2;
        return distance;
    }
    public double getAverageWEquation(int num){
        double[] distances = new double[num];
        for (int i = 0; i < num; i++) {
            distances[i] = getVoltageWEquation();
        }
        return calculateAverage(distances);
    }



    public double avgOf(int num) {
        double[] distances = new double[num];
        for (int i = 0; i < num; i++) {
            distances[i] = getVoltageWEquation();
        }
        return calculateAverage(distances);
    }

    private double calculateAverage(double[] values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }
    public double getRightDistance(){
        double distance = (rightUS.getVoltage()/((3.3/1024)*4.5));
        return distance;
    }

}
