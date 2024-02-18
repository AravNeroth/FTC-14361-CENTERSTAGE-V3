package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths.StateMachines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.currentState;

@Autonomous(name = "closeBlueState", group = "Auto")
public class closeBlueState extends LinearOpMode {
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    NewVision newVision;


    String webcamName;
    Robot bot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime temporalMarkerTimer = new ElapsedTime();
    Pose2d start = new Pose2d(11.5, -62.75, Math.toRadians(270));
    SampleMecanumDrive drive;
    String selection;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, finishBoard = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double detectYPos = 0, detectYNeg = 0;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 4;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    HSVBlueDetection blueDetection;

    currentState currentStates;
    boardState boardStates;
    Pose2d boardPose, parkPose;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));


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
                .lineToConstantHeading(new Vector2d(22.25,55))
                .lineToConstantHeading(new Vector2d(22.25,42))
                .lineToConstantHeading(new Vector2d(22.25,50))
                .lineToLinearHeading(new Pose2d(42 ,42.5, Math.toRadians(180)))
                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10, 38))
                .lineToLinearHeading(new Pose2d(40 ,38, Math.toRadians(180)))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15, 54))
                .lineToLinearHeading(new Pose2d(15,32, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(8.5,32))
                .lineToConstantHeading(new Vector2d(14,32))
                .lineToLinearHeading(new Pose2d(30,30,Math.toRadians(180)))
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
                .lineToConstantHeading(new Vector2d(53,31))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(52.8,31))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(43,31))
                .build();

        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(boardPose)
                .lineToConstantHeading(new Vector2d(53,31))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(52.8,31))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(43,31))
                .build();

        // ---------------------------- Park ---------------------------- //

        TrajectorySequence park = drive.trajectorySequenceBuilder(parkPose)
                .lineToConstantHeading(new Vector2d(47, 32))
                .lineToLinearHeading(new Pose2d(47 ,58, Math.toRadians(270)))
                .build();

        // ------------------------ Runner ------------------------- //

        waitForStart();
     //   camera.stopStreaming();
        if (isStopRequested()) return;
        switch (newVision.getStartingPosition())
        {
            case LEFT:
                drive.followTrajectorySequence(leftTape);
                boardStates = boardState.left;
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightTape);
                boardStates = boardState.right;
                break;
            case CENTER:
                drive.followTrajectorySequence(centerTape);
                boardStates = boardState.center;
                break;
        }
        switch (currentStates)
        {
            case tape:

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

    private void newColorDetect(){
        if(initCam){
            visionPortal.stopStreaming();
        }
        else
        {
            newVision = new NewVision(telemetry);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(newVision)
                    // .setCamera(BuiltinCameraDirection.BACK)

                    //  .enableLiveView(false)
                    // .addProcessor(newVision)
                    .build();

//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), newVision);

            //  NewVision.StartingPosition startingPos = NewVision.StartingPosition.LEFT;
            visionPortal.resumeStreaming();
//            telemetry.addLine("vision portal built");
//            telemetry.addData("starting position: ", startingPos);
//            startingPos = newVision.getStartingPosition();
            //     telemetry.addData("called NewVision- returned: ", startingPos);
            initCam = true;
        }


    }
}