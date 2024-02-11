package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "Distance Testing ", group = "goobTest")
public class distanceTesting extends LinearOpMode {

    distanceSensor distSensor;
    Robot bot;
    SampleMecanumDrive drive;
    Pose2d startPose;

    // True is blue, false is red. DONT TOUCH JUST ENTER IT IN!

    boolean setBlue = true;
    boolean setRed = false;

    public void runOpMode() {

        distSensor = new distanceSensor(hardwareMap, setBlue);

        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-33, 61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        telemetry.addLine("Sensor & Drivetrain Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            distanceTelemetry();
            telemetry.update();
        }
    }


    public void distanceTelemetry(){
        // if true, then blue. if false, red detect
        if(distSensor.getAlliance())
            telemetry.addLine("Using Sensor: LEFT");
        else
            telemetry.addLine("Using Sensor: RIGHT");

        telemetry.addData("Distance To Wall: ", distSensor.getDistance());
    }
}