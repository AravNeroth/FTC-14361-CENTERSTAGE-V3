package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;

@Autonomous(name = "Lid distance testing ", group = "goobTest")
public class lidDistanceTester extends LinearOpMode {

    distanceSensor distSensor;

    public void runOpMode() {

        distSensor = new distanceSensor(hardwareMap);
        telemetry.addLine("Sensor Status: Initialized.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            backBoardDistanceTelemetry();
            telemetry.update();


        }

    }


    public void backBoardDistanceTelemetry(){

        if(distSensor.getDistance() < 3.5){
            telemetry.addLine("PIXEL DETECTED!");
            telemetry.addLine("Space Between Sensor & Pixel: " + distSensor.getDistance());
        }

        else
            telemetry.addLine("Space Between Sensor & Board: " + distSensor.getDistance());
    }
}