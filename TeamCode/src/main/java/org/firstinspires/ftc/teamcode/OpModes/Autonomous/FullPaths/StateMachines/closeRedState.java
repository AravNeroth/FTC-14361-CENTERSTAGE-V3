package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths.StateMachines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.currentState;

@Autonomous(name = "closeRedState", group = "Auto")
public class closeRedState extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;
    currentState currentStates;
    boardState boardStates;
    Pose2d boardPose, parkPose;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(270));
        initCam();

        drive.setPoseEstimate(startPose);
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        bot.setLidPosition(lidState.close);

        currentStates = currentState.tape;

        // ---------------------------- Tape ---------------------------- //

        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(17, -54))
                .lineToLinearHeading(new Pose2d(17,-33, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(12,-33))
                .lineToConstantHeading(new Vector2d(15,-33))
                .lineToLinearHeading(new Pose2d(30,-28.5, Math.toRadians(180)))
                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(17,-55))
                .lineToConstantHeading(new Vector2d(17, -38))
                .lineToLinearHeading(new Pose2d(40 ,-34, Math.toRadians(180)))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22.5,-39))
                .lineToConstantHeading(new Vector2d(22.25,-50))
                .lineToLinearHeading(new Pose2d(42 ,-35, Math.toRadians(180)))
                .build();

        // ---------------------------- Board ---------------------------- //

        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(boardPose)
                .lineToConstantHeading(new Vector2d(52,42.5))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(51.8,42.5))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(43, 42.5))
                .build();

        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(boardPose)
                .lineToConstantHeading(new Vector2d(55,-32))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(54.8, -32))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(43, -32))
                .build();

        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(boardPose)
                .lineToConstantHeading(new Vector2d(55,-28.5))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(54.8,-28.5))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(47,-28.5))
                .build();

        // ---------------------------- Park ---------------------------- //

        TrajectorySequence park = drive.trajectorySequenceBuilder(parkPose)
                .lineToLinearHeading(new Pose2d(50, -53, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(52, -54.5))
                .build();

        // ------------------------ Runner ------------------------- //

        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (currentStates)
        {
            case tape:
                switch (redDetection.getLocation())
                {
                    case LEFT:
                        drive.followTrajectorySequence(leftTape);
                        boardStates = boardState.left;
                        break;
                    case RIGHT:
                        drive.followTrajectorySequence(rightTape);
                        boardStates = boardState.right;
                        break;
                    case MIDDLE:
                        drive.followTrajectorySequence(centerTape);
                        boardStates = boardState.center;
                        break;
                }
                currentStates = currentState.board;
                break;

            case board:
                bot.outtakeSlide.setPosition(500);
                bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                bot.setWristPosition(wristState.outtaking);

                if(boardStates == boardState.left)
                {
                    boardPose = leftTape.end();
                    drive.followTrajectorySequence(leftBoard);
                    parkPose = leftBoard.end();
                }
                else if(boardStates == boardState.center)
                {
                    boardPose = centerTape.end();
                    drive.followTrajectorySequence(centerBoard);
                    parkPose = centerBoard.end();
                }
                else
                {
                    boardPose = rightTape.end();
                    drive.followTrajectorySequence(rightBoard);
                    parkPose = rightBoard.end();
                }
                currentStates = currentState.park;
                break;

            case park:
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);

                drive.followTrajectorySequence(park);
                break;
        }
    }

    // ------------------------ Camera Initialization ------------------------- //

    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        redDetection = new HSVRedDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(redDetection);

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