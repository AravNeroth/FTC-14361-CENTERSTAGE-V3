package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.currentState;
import org.firstinspires.ftc.teamcode.Commands.frontDistanceState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "simpleDD", group = "Auto")
public class simpleDistanceDetect extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    distanceSensor frontDistanceSensor;
    currentState currentStates;
    boolean leftSide, centerSide, rightSide;
    String webcamName;
    Robot bot;
    Pose2d endTapePose, stackEndPose;
    frontDistanceState frontDistanceStates;
    int tapeAttempts = 0;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        frontDistanceSensor = new distanceSensor(hardwareMap);
        currentStates = currentState.stack1;

        waitForStart();
        if (isStopRequested()) return;

        switch (currentStates){
            case stack1:
                boolean aligned = false;
                frontDistanceStates = frontDistanceSensor.frontAdjust();

                switch (frontDistanceStates) {
                    case Far:
                        TrajectorySequence stackForwardAdjust = drive.trajectorySequenceBuilder(startPose)
                                .forward(3)
                                .build();

                        drive.followTrajectorySequence(stackForwardAdjust);

                        stackEndPose = stackForwardAdjust.end();
                        break;
                }
        }
    }
}