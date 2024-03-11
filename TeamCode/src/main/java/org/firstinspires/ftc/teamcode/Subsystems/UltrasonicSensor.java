package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UltrasonicSensor {

    private AnalogInput analog;
    // private int maxR = 520;

    public UltrasonicSensor(HardwareMap hardwareMap) {
        analog = hardwareMap.get(AnalogInput.class, "UltraSonicSensor");
    }

    public double getDistance() {
        double distance = (analog.getVoltage()/((3.3/1024)*6));
        return distance;
    }

}
