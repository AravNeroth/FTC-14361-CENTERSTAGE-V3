package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Spline Testing", group = "AravTest")

public class SpineTesting extends LinearOpMode {
    Robot bot;
    SampleMecanumDrive drive;
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        // init stuff below
        timer.reset();
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(90)));

        // not gonna lie, this is prob not gonna even work lmao but let's see!
        Trajectory TrussSplineTest = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(81))
                .splineToSplineHeading(new Pose2d(-26, -37, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(2, -36, Math.toRadians(0)), Math.toRadians(27))
                .splineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)), Math.toRadians(319))
                // make first angle to ‘0’ if 180 does not work
                .splineToSplineHeading(new Pose2d(49, -35, Math.toRadians(180)), Math.toRadians(0))
                .build();

        // this has a higher chance at working than the TrussSpline (which isn't saying much)
        Trajectory LongRedSplinePark = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(113))
                .splineToSplineHeading(new Pose2d(-54, -31, Math.toRadians(90)), Math.toRadians(65))
                .splineToSplineHeading(new Pose2d(-33, -10, Math.toRadians(90)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-7, -5, Math.toRadians(90)), Math.toRadians(46))
                .splineToSplineHeading(new Pose2d(18, -20, Math.toRadians(90)), Math.toRadians(335))
                .splineToSplineHeading(new Pose2d(40, 5, Math.toRadians(90)), Math.toRadians(30))
                .splineToSplineHeading(new Pose2d(60, -11, Math.toRadians(90)), Math.toRadians(345))
                .build();

        telemetry.addLine("Spline Paths Initialized w/ No Errors.");
        telemetry.addData("Total Time Taken: ", timer.time(TimeUnit.SECONDS) + " Seconds.");
        telemetry.addLine("Start Robot at LongRed. It should spline through gate to go to backboard.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        // below will run stuff

        drive.followTrajectory(LongRedSplinePark);

    }
}