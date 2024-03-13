package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UltrasonicSensor {

    private AnalogInput leftUS, rightUS;
    // private int maxR = 520;

    public UltrasonicSensor(HardwareMap hardwareMap) {
        leftUS = hardwareMap.get(AnalogInput.class, "leftUltraSonicSensor");
     //   rightUS = hardwareMap.get(AnalogInput.class, "rightUltraSonicSensor");
    }

    public double getLeftDistance() {
        double distance = (leftUS.getVoltage()/((3.3/1024)*4.5))-3;
        return distance;
    }
    public double getVoltage(){
        double voltage = leftUS.getVoltage();
        return voltage;
    }
    public double getVoltageWEquation(){
        double distance = 88.7*(leftUS.getVoltage())-15.2 + 3.2;
        return distance;
    }
    public double getAverageWEquation(int num){
        double[] distances = new double[num];
        for (int i = 0; i < num; i++) {
            distances[i] = getVoltageWEquation();
        }
        return calculateAverage(distances);
    }

    public double getDistance() {
        return sqrtModelN(leftUS.getVoltage());
    }

    private double sqrtModelN(double voltage) {
        return -108.795 * Math.sqrt(-.0237324 * (voltage - 206.937)) + 276.65;
    }

    public double avgOf(int num) {
        double[] distances = new double[num];
        for (int i = 0; i < num; i++) {
            distances[i] = getLeftDistance();
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
