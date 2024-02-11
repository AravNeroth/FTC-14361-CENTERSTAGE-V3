package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "stack distance testing ", group = "goobTest")
public class distanceTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }

//    DistanceSensor distSensor;
//    Robot bot;
//    SampleMecanumDrive drive;
//    Pose2d startPose;
//
//
//    // true is blue, false is red. DONT TOUCH JUST ENTER IT IN!
//    boolean setBlue = true;
//    boolean setRed = false;
//
//    public void runOpMode() {
//
//        distSensor = new DistanceSensor(hardwareMap);
//
//        bot = new Robot(hardwareMap, telemetry);
//        drive = new SampleMecanumDrive(hardwareMap);
//        startPose = new Pose2d(-33, 61, Math.toRadians(90));
//
//        drive.setPoseEstimate(startPose);
//
//        telemetry.addLine("Sensor & Drivetrain Initialized");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            distanceTelemetry();
//            telemetry.update();
//
//
//        }
//
//    }
//
//
//    public void distanceTelemetry(){
//        // if true, then blue. if false, red detect
//        if(distSensor.getAlliance())
//            telemetry.addLine("Using Sensor: LEFT");
//        else
//            telemetry.addLine("Using Sensor: RIGHT");
//
//        telemetry.addData("Distance To Wall: ", distSensor.getDistance());
//    }
}