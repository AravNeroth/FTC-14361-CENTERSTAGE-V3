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

@Autonomous(name = "Lid distance testing ", group = "goobTest")
public class lidDistanceTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }


//    DistanceSensor distSensor;
//
//    public void runOpMode() {
//
//        distSensor = new DistanceSensor(hardwareMap);
//        telemetry.addLine("Sensor Status: Initialized.");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            backBoardDistanceTelemetry();
//            telemetry.update();
//
//
//        }
//
//    }
//
//
//    public void backBoardDistanceTelemetry(){
//
//        if(distSensor.getDistance() < 3.5){
//            telemetry.addLine("PIXEL DETECTED!");
//            telemetry.addLine("Space Between Sensor & Pixel: " + distSensor.getDistance());
//        }
//
//        else
//            telemetry.addLine("Space Between Sensor & Board: " + distSensor.getDistance());
//    }
}