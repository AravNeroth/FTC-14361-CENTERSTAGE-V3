package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.currentState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.stackState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(name = "closeBlueStackCD", group = "Auto")
public class closeStackColorDetection extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    ColorSensor colorSense;
    currentState currentStates;
    boolean leftSide, centerSide, rightSide;
    String webcamName;
    Robot bot;
    Pose2d endTapePose, stackEndPose;
    stackState stackStates;
    int tapeAttempts = 0;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        currentStates = currentState.tape;
        initCam();

        // ---------------------------- Left Tape ---------------------------- //

        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22.25,55))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .lineToConstantHeading(new Vector2d(22.25,42))
                .lineToConstantHeading(new Vector2d(22.25,50))
                .lineToLinearHeading(new Pose2d(48 ,42.5, Math.toRadians(180)))
                .build();

        // ---------------------------- Center Tape ---------------------------- //

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10,55))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .lineToConstantHeading(new Vector2d(10, 33.5))
                .lineToConstantHeading(new Vector2d(10, 38))
                .lineToLinearHeading(new Pose2d(40 ,38, Math.toRadians(180)))
                .build();

        // ---------------------------- RightTape ---------------------------- //

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15, 54))

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .lineToLinearHeading(new Pose2d(15,32, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(8.5,32))
                .lineToConstantHeading(new Vector2d(14,32))
                .lineToLinearHeading(new Pose2d(30,30,Math.toRadians(180)))
                .build();

        // ---------------------------- Gate ---------------------------- //

        TrajectorySequence gate = drive.trajectorySequenceBuilder(endTapePose)
                .lineToConstantHeading(new Vector2d(40, 11))
                .lineToConstantHeading(new Vector2d(-40, 11))
                .lineToConstantHeading(new Vector2d(-40, 20))
                .build();

        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (currentStates)
        {
            case tape:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);

                switch(blueDetection.getLocation())
                {
                    case LEFT:
                        drive.followTrajectorySequence(leftTape);
                        leftSide = true;
                        break;

                    case RIGHT:
                        drive.followTrajectorySequence(rightTape);
                        rightSide = true;
                        break;

                    case MIDDLE:
                        drive.followTrajectorySequence(centerTape);
                        centerSide = true;
                        break;
                }
                currentStates = currentState.gate;
                //Skipping board scoring now. This will include april tag detection later.
                break;
//            case board:
//                if(leftSide)
//                {
//                    endTape = leftTape.end();
//                }
//                else if (centerSide)
//                {
//                    endTape =  centerTape.end();
//                }
//                else
//                {
//                    endTape = rightTape.end();
//                }
//
//                currentStates = currentState.gate;
//                break;
            case gate:
                if(leftSide)
                {
                    endTapePose = leftTape.end();
                }
                else if (centerSide)
                {
                    endTapePose =  centerTape.end();
                }
                else
                {
                    endTapePose = rightTape.end();
                }

                drive.followTrajectorySequence(gate);

                currentStates = currentState.stack1;
                break;
            case stack1:
                boolean aligned = false;

                while(aligned = false)
                {
                    stackStates = colorSense.align(tapeAttempts);

                    switch (stackStates)
                    {
                        case offLeft:
                            TrajectorySequence stackAdjustRight = drive.trajectorySequenceBuilder(gate.end())
                                    .strafeRight(.75)
                                    .build();

                            drive.followTrajectorySequence(stackAdjustRight);


                            stackEndPose = stackAdjustRight.end();
                            break;
                        case offRight:
                            TrajectorySequence stackAdjustLeft = drive.trajectorySequenceBuilder(gate.end())
                                    .strafeLeft(.75)
                                    .build();

                            drive.followTrajectorySequence(stackAdjustLeft);

                            stackEndPose = stackAdjustLeft.end();
                            break;
                        case midAligned:
                            aligned = true;
                            break;
                    }
                }

                TrajectorySequence stackIntake = drive.trajectorySequenceBuilder(gate.end())
                        .lineToLinearHeading(new Pose2d(-60, 19, Math.toRadians(200)))
                        .lineToLinearHeading(new Pose2d(-54, 11, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            bot.setActiveIntakePosition(activeIntakeState.active);
                        })
                        .lineToConstantHeading(new Vector2d(-61, 11))
                        .addDisplacementMarker(() -> {
                            bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                        })
                        .lineToConstantHeading(new Vector2d(-50, 11))
                        .build();

                drive.followTrajectorySequence(stackIntake);
                
                break;
        }
    }
    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        blueDetection = new HSVBlueDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(blueDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.showFpsMeterOnViewport(true);
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
}