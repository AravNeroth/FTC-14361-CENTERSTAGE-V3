package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.opmode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleVoltMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.TwoWheelTrackingLocalizerVolt;

import ftc.rogue.blacksmith.util.kalman.KalmanFilter;


public class KalmanTwoWheelTrackingLocalizerVolt extends TwoWheelTrackingLocalizerVolt {

    private final KalmanFilter headingFilter, wheelPos1Filter, wheelPos2Filter, wheelPos3Filter, headingVelocityFilter, wheelPos1VelocityFilter, wheelPos2VelocityFilter, wheelPos3VelocityFilter;

    // Below is part of VoltMecanum code btw- basically in order to keep the old sample mecanum
    // AND have a volt one (in case volt breaks), you need to send the VoltMecanum object to a few places.

    /*
    so in this file, depending on what object you use, Sample or Volt, it will redirect the object to
    the T
    woWheelTrackingLocalizer, or the TwoWheelVoltTrackingLocalizer. Both the files are exactly the same,
    but they only take in 1 drivetrain object if that makes any sense.
     */
    public KalmanTwoWheelTrackingLocalizerVolt(HardwareMap hardwareMap, SampleVoltMecanumDrive drive)
    {
        super(hardwareMap, drive);

        // The KalmanFilter R and Q values refer to how you rely on either your sensor output or prediction output.

        // The process noise (R) refers to how much noise is in the actual system.
        //This characterizes the system, it's what's actually happening in the real world.
        // This can be caused by a ever-so-slight jitter in the motor or fluctuations in battery voltage,
        // or any number of things actually.

        // The measurement noise (Q) is how much noise is in the system output. This is caused due to slight
        // inaccuracies that are inherent in all sensors/systems, and can become pretty significant over time.

        headingFilter = new KalmanFilter(0,0);
        wheelPos1Filter = new KalmanFilter(0,0);
        wheelPos2Filter = new KalmanFilter(0,0);
        wheelPos3Filter = new KalmanFilter(0,0);
        headingVelocityFilter = new KalmanFilter(0,0);
        wheelPos1VelocityFilter = new KalmanFilter(0,0);
        wheelPos2VelocityFilter = new KalmanFilter(0,0);
        wheelPos3VelocityFilter = new KalmanFilter(0,0);
    }
}
