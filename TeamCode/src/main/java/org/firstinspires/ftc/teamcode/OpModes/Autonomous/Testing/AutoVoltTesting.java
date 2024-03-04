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

import java.text.DecimalFormat;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Voltage Testing", group = "AravTest")

public class AutoVoltTesting extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    Double m1Vel, m2Vel, m3Vel, m4Vel;
    List<Double> allMotorSpeed;

    // init stuff below
        public static double DISTANCE = 50; // in - robot will go AROUND this much! allocate space

        @Override
        public void runOpMode () throws InterruptedException {
            timer.reset();
            DecimalFormat numberFormat = new DecimalFormat("#.00");

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            SampleVoltMecanumDrive drive = new SampleVoltMecanumDrive(hardwareMap);

            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(DISTANCE)
                    .build();

            telemetry.addLine("Instructions: Grab A Tape Measure And Give Bot Room To Go Straight.");
            telemetry.addLine("Measure & Note Distance On | V14+ | V13.9 | V13.8 | V13.7 | V13.6 |");
            telemetry.addLine("Adjust 'mult' in SampleVoltMecanumDrive setPower() until robot travels same dist no matter battery.");
            telemetry.addLine("Voltage on Initialization Completion: " + drive.getBatteryVoltage());
            telemetry.addLine("Ready to Start.");

            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory);


            telemetry.addLine("Voltage: " + drive.getBatteryVoltage());
            telemetry.addLine("Current Mult: " + drive.getPowerMult());

            allMotorSpeed = drive.getWheelPowers();
            telemetry.addLine("Motor Powers: | LF: " + (allMotorSpeed.get(0)) + "| LR: " + (allMotorSpeed.get(1)) + "| RR: " + (allMotorSpeed.get(2)) + "| RF:  " + (allMotorSpeed.get(3)) + "|" );

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("final X Position: ", poseEstimate.getX());
            telemetry.addData("final Y Position: ", poseEstimate.getY());
            telemetry.addData("finalHeading (although u don't need this): ", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }

}