package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.currentState;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

    ColorSensor leftColorSensor, rightColorSensor;

    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        colorSense = new colorSensor(hardwareMap);

        telemetry.addLine("Sensor & Drivetrain Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested())
        {
            telemetry.addLine("Left Color Sensor Red: " + colorsense.getLeftRedVal());
            telemetry.addLine("Left Color Sensor Green: " + colorTelemetry());
            telemetry.addLine("Left Color Sensor Blue: " + colorTelemetry());

            telemetry.addLine("Right Color Sensor Red: " + rightColorSensor.red());
            telemetry.addLine("Right Color Sensor Green: " + rightColorSensor.green());
            telemetry.addLine("Right Color Sensor Blue: " + rightColorSensor.blue());

            telemetry.update();
        }
    }

    public String colorTelemetry()
    {
        if(leftColorSensor.red() > 255 && leftColorSensor.green() > 255 && leftColorSensor.blue() > 255)
        {
            return "white";
        }
        return "not Left White";
    }
}