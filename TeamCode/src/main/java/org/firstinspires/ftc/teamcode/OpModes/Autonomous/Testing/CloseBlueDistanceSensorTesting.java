package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Close Blue Distance Sensor Testing", group = "goobTest")
public class CloseBlueDistanceSensorTesting extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
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
    Pose2d start = new Pose2d(11.5, 62.75, Math.toRadians(90));
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
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, BLUESTACK = 9;
    int ID_TAG_OF_INTEREST = 4;
    boolean tagFound = false;
    double offset = 3;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    state currentState = state.tape;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack, idle, park
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                //.lineToConstantHeading(new Vector2d(19,-55))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(11.5, 34))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(11.5, 40))

                .lineToLinearHeading(new Pose2d(25 ,40, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .build();
        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(15, 40))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(15,33, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(9, 33))
                .lineToConstantHeading(new Vector2d(15, 33))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(25 ,33, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .build();
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(22,45))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })


                //     .back(5)
                .lineToConstantHeading(new Vector2d(22 ,50))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence goToCenterAprilTag = drive.trajectorySequenceBuilder(centerTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(25, 32))

                //   .strafeRight(3)
                .build();
        TrajectorySequence goToLeftAprilTag = drive.trajectorySequenceBuilder(leftTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(25, 34))
                //   .strafeRight(3)
                .build();
        TrajectorySequence goToRightAprilTag = drive.trajectorySequenceBuilder(rightTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(30, 24))

                //   .strafeRight(3)
                .build();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());



        telemetry.update();

        TrajectorySequence score = null;
        waitForStart();


        if (isStopRequested()) return;
        NewVision.StartingPosition startPath = newVision.getStartingPosition();
        switch (newVision.getStartingPosition()){
            case CENTER:
                ID_TAG_OF_INTEREST = MIDDLE;
                temporalMarkerTimer.reset();
                drive.followTrajectorySequenceAsync(centerTape);
                telemetry.addLine("CENTER.");
                telemetry.update();

                break;
            case RIGHT:
                ID_TAG_OF_INTEREST = RIGHT;
                temporalMarkerTimer.reset();

                drive.followTrajectorySequenceAsync(rightTape);
                telemetry.addLine("right.");
                telemetry.update();

                break;
            case LEFT:
                ID_TAG_OF_INTEREST = LEFT;
                temporalMarkerTimer.reset();

                drive.followTrajectorySequenceAsync(leftTape);
                telemetry.addLine("left.");
                telemetry.update();

                break;
        }
        currentState = state.tape;
        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case tape:
                    if (!cameraOn) {
                        // newColorDetect();

                        telemetry.addLine("Into disalbe");
                        telemetry.update();
                        cameraOn = true;
                        timer.reset();

                    }
                    if (!aprilTagOn) {
                        telemetry.addLine("into april tag enable");
                        telemetry.update();
                        initAprilTag();
                        aprilTagOn = true;
                    }


                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(new Pose2d(72-bot.distanceSensor.getBotsRightCenterDistance(), drive.getPoseEstimate().getY(), Math.toRadians(180)));
                        currentState = state.firstTimeBoard;
                        temporalMarkerTimer.reset();
                        timer.reset();
                    }
//
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case firstTimeBoard:
                    if(!drive.isBusy()){
                        switch (startPath){
                            case RIGHT:
                                score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToConstantHeading(new Vector2d(62.5, 32))
                                        .addTemporalMarker(.05,() -> {
                                          bot.outtakeSlide.setPosition(525);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(700);
                                        })
                                        .build();
                                drive.followTrajectorySequenceAsync(score);
                                currentState = state.park;
                                break;
                            case CENTER:
                                score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToConstantHeading(new Vector2d(62.5, 38))
                                        .addTemporalMarker(.05,() -> {
                                            bot.outtakeSlide.setPosition(515);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(700);
                                        })
                                        .build();
                                drive.followTrajectorySequenceAsync(score);
                                currentState = state.park;
                                break;
                            case LEFT:
                                score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToConstantHeading(new Vector2d(62.5, 44))
                                        .addTemporalMarker(.05,() -> {
                                            bot.outtakeSlide.setPosition(525);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(700);
                                        })
                                        .build();

                                drive.followTrajectorySequenceAsync(score);
                                drive.setPoseEstimate(new Pose2d(52, drive.getPoseEstimate().getY(), Math.toRadians(180)));
                                currentState = state.park;
                                break;

                        }
                    }

                                      break;
                case park:
                    if(!drive.isBusy()){
                        TrajectorySequence parkNextToBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(5)
                                .addDisplacementMarker(5,  () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(10,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

                                })

                                .lineToLinearHeading(new Pose2d(46, 12, Math.toRadians(270)))

//

                                .lineToConstantHeading(new Vector2d(52, 12))
                                .build();
                        TrajectorySequence parkInCorner = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(5)
                                .addDisplacementMarker(5,  () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(10,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

                                })
                                .lineToLinearHeading(new Pose2d(46, 55, Math.toRadians(270)))

//

                                .lineToConstantHeading(new Vector2d(52, 55))
                                .build();
                        drive.followTrajectorySequenceAsync(parkInCorner);
                        currentState = state.idle;
                    }
                case idle:
                    telemetry.addLine("Inside Idle State");
              //      telemetry.addData("Tag ID", tagOfInterest.id);
            //        telemetry.addData("pose est ", drive.getPoseEstimate());
           //         telemetry.addData("Tag y", tagY);
           //         telemetry.addData("double y + ", detectYPos);
              //      telemetry.addData("double y -", detectYNeg);
               //     telemetry.addData("tag ", tagOfInterest.metadata.fieldPosition.get(1)+5);

                    telemetry.update();
                    break;
            } //switch statement end

            drive.update();
        } // opmode loop

    } // run opmode



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
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);


        // Create the vision portal by using a builder.

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .enableLiveView(false)
                    .addProcessors(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}

