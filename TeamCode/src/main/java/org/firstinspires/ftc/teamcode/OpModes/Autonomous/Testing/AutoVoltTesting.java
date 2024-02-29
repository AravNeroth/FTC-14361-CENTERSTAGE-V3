package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleVoltMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Voltage Testing", group = "AravTest")

public class AutoVoltTesting extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    // init stuff below
        public static double DISTANCE = 70; // in

        @Override
        public void runOpMode () throws InterruptedException {
            timer.reset();

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            SampleVoltMecanumDrive drive = new SampleVoltMecanumDrive(hardwareMap);

            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(DISTANCE)
                    .build();

            telemetry.addLine("Instructions: Grab A Tape Measure And Give Bot Room To Go Straight.");
            telemetry.addLine("Measure & Note Distance On | V14+ | V13.9 | V13.8 | V13.7 | V13.6 |");
            telemetry.addLine("Adjust 'mult' in SampleVoltMecanumDrive file until robot travels same dist no matter battery.");
            telemetry.addLine("Ready to Start.");

            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory);

            telemetry.addLine("Current Voltage: " + drive.getBatteryVoltage());
            telemetry.addLine("Motor Speeds: ");


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("final X Position: ", poseEstimate.getX());
            telemetry.addData("final Y Position: ", poseEstimate.getY());
            telemetry.addData("finalHeading (although u don't need this): ", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }

}