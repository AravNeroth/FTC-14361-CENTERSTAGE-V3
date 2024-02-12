package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.currentState;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "colorSensorTest", group = "Auto")
public class colorSensorTest extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    colorSensor colorSense;
    currentState currentStates;
    String webcamName;
    Robot bot;

    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        colorSense = new colorSensor(hardwareMap);

        drive.setPoseEstimate(startPose);

        telemetry.addLine("Sensor & Drivetrain Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested())
        {
            colorTelemetry();
            telemetry.update();
        }
    }

    public void colorTelemetry()
    {
        telemetry.addLine("Left Color Sensor Red: " + colorSense.getLeftRedVal());
        telemetry.addLine("Left Color Sensor Green: " + colorSense.getLeftGreenVal());
        telemetry.addLine("Left Color Sensor Blue: " + colorSense.getLeftBlueVal());

        telemetry.addLine("Right Color Sensor Red: " + colorSense.getRightRedVal());
        telemetry.addLine("Right Color Sensor Green: " + colorSense.getRightGreenVal());
        telemetry.addLine("Right Color Sensor Blue: " + colorSense.getRightBlueVal());

    }
}