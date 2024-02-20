package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.linkageState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Autonomous(name = "Long Blue Distance Sensor ", group = "goobTest")
public class LongBlueDistanceSensor extends LinearOpMode {

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
    Pose2d start = new Pose2d(-36.5, 62.75, Math.toRadians(90));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, underTrussBool = false, stackBool = false, toboard = false,finishBoard = false, lineUp = false, aligned = false, LeaveBackboard = false, lineUp2 = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    double centerTagX = 60.275, centerTagY = 29.4;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 2;
    double offset = 0;
    double batteryOffset = 0;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

   state currentState = state.tape;

    enum state {
        tape, underTruss,firstTimeBoard, secondTimeBoard, thirdTimeBoard, toStack, idle,leaveStack, park, scoring, leaveBackboard, underTrussFromBackboard
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)

                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToConstantHeading(new Vector2d(-38.25, 36.4))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-38.25, 40))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                .lineToLinearHeading(new Pose2d(-38.25, 60, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-46.75, 43))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-46.75, 55))
                .lineToLinearHeading(new Pose2d(-38.25, 60, Math.toRadians(180)))
//                .addTemporalMarker(() -> {
//                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
//                    bot.setWristPosition(wristState.outtaking);
//                })
                .build();
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 90, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-39.75, 35, Math.toRadians(180)))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToConstantHeading(new Vector2d(-36.75, 35))
                .lineToConstantHeading(new Vector2d(-39.75, 35))
                .lineToConstantHeading(new Vector2d(-38.25, 60))
//                .addTemporalMarker(() -> {
//                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
//                    bot.setWristPosition(wristState.outtaking);
//                })
//
                .build();

        TrajectorySequence underTrussToStack = drive.trajectorySequenceBuilder(new Pose2d(centerTagX - 8, centerTagY, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(40, centerTagY - 6))
                .addTemporalMarker(.25, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })
                .addTemporalMarker(.4, () -> {

                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40, -53))
                .lineToConstantHeading(new Vector2d(-40, -53))
                .addTemporalMarker(() -> {
                    bot.setLinkagePosition(linkageState.HIGH);

                })
                //  .resetVelConstraint()
                // .lineToConstantHeading(new Vector2d(-60, -28.5))
                .lineToConstantHeading(new Vector2d(-59.9, -33))

                //  .lineToLinearHeading(new Pose2d(-60,-28, Math.toRadians(230)))

                //  .forward(.5)
                //  .turn(Math.toRadians(-100))
                .strafeRight(5)

                // .forward(1)
                .waitSeconds(.05)


//                .lineToConstantHeading(new Vector2d(50, -61))

                .resetVelConstraint()
                //   .strafeRight(3)
                //   .strafeRight(3)
                .build();
        TrajectorySequence underTrussToStackLeftTag = drive.trajectorySequenceBuilder(new Pose2d(centerTagX - 8, centerTagY + 6, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(40, centerTagY + 6))
                .addTemporalMarker(.25, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })
                .addTemporalMarker(.4, () -> {

                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40, 53))
                .lineToConstantHeading(new Vector2d(-40, 53))
                .addTemporalMarker(() -> {
                    bot.setLinkagePosition(linkageState.HIGH);

                })
                //  .resetVelConstraint()
                // .lineToConstantHeading(new Vector2d(-60, -28.5))
                .lineToConstantHeading(new Vector2d(-59.9, 33))

                //  .lineToLinearHeading(new Pose2d(-60,-28, Math.toRadians(230)))

                //  .forward(.5)
                //  .turn(Math.toRadians(-100))
                .strafeRight(5)

                // .forward(1)
                .waitSeconds(.05)


//                .lineToConstantHeading(new Vector2d(50, -61))

                .resetVelConstraint()
                //   .strafeRight(3)
                //   .strafeRight(3)
                .build();
        TrajectorySequence underTrussToStackRightTag = drive.trajectorySequenceBuilder(new Pose2d(centerTagX - 8, centerTagY - 6, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(40, centerTagY - 6))
                .addTemporalMarker(.25, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })
                .addTemporalMarker(.4, () -> {

                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40, -53))
                .lineToConstantHeading(new Vector2d(-40, -53))
                .addTemporalMarker(() -> {
                    bot.setLinkagePosition(linkageState.HIGH);

                })
                //  .resetVelConstraint()
                // .lineToConstantHeading(new Vector2d(-60, -28.5))
                .lineToConstantHeading(new Vector2d(-59.9, -33))

                //  .lineToLinearHeading(new Pose2d(-60,-28, Math.toRadians(230)))

                //  .forward(.5)
                //  .turn(Math.toRadians(-100))
                .strafeRight(5)

                // .forward(1)
                .waitSeconds(.05)


//                .lineToConstantHeading(new Vector2d(50, -61))

                .resetVelConstraint()
                //   .strafeRight(3)
                //   .strafeRight(3)
                .build();
        TrajectorySequence leaveStack = drive.trajectorySequenceBuilder((underTrussToStack.end()))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-59.5, -33.5))
                .addDisplacementMarker(1, () -> {
                    bot.setLinkagePosition(linkageState.LOW);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                //  .waitSeconds(.25)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-55, -28.5, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .back(3)
                .addTemporalMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
                .lineToConstantHeading(new Vector2d(-40, -56))
                .addTemporalMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(30, -55))
                .addDisplacementMarker(() -> {
                    bot.setArmState(armState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(32.5, -30))


                .resetVelConstraint()

                .build();

        //   .strafeRight(3)


//        TrajectorySequence goToCenterAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
//                .lineToConstantHeading(new Vector2d(32.5, 41))
//
//                .resetVelConstraint()
//
//                //   .strafeRight(3)
//                .build();
//        TrajectorySequence goToLeftAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
//                .lineToConstantHeading(new Vector2d(32.5, 46))
//
//                .resetVelConstraint()
//
//                //   .strafeRight(3)
//                .build();
//        TrajectorySequence goToRightAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
//                .lineToConstantHeading(new Vector2d(32.5, 36))
//
//                .resetVelConstraint()
//
//                //   .strafeRight(3)
//                .build();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());


        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
        switch (newVision.getStartingPosition()) {
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

                        currentState = state.underTruss;
                        temporalMarkerTimer.reset();
                        timer.reset();
                    }


//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case underTruss:
                    telemetry.addData("Pose Est", drive.getPoseEstimate());
                    if (!drive.isBusy() && !lineUp) {
                        TrajectorySequence lineUpWithTrussWall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.25))


                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequence(lineUpWithTrussWall);
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 - bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                        lineUp = true;
                    }
                    if(!drive.isBusy()){
                        if(!underTrussBool){
                            TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                                    .lineToConstantHeading(new Vector2d(30, drive.getPoseEstimate().getY()))
//                                    .addTemporalMarker(() -> {
//                                        bot.setArmPosition(armState.outtaking, armExtensionState.extending);
//                                        bot.setWristPosition(wristState.outtaking);
//                                    })
                                    .waitSeconds(.05)
                                    .resetVelConstraint()
                                    //   .strafeRight(3)
                                    .build();
                            drive.followTrajectorySequenceAsync(underTruss);
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 -  bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                            underTrussBool = true;

                        }
                    }

                    if (!drive.isBusy()) {
                        currentState = state.firstTimeBoard;
                    timer.reset();

        }
        break;
                case firstTimeBoard:
                    if(!drive.isBusy() && !toboard) {
                        TrajectorySequence toBackBoard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(30,39.25))
                                .lineToConstantHeading(new Vector2d(55,39.25))
                             //   .splineToConstantHeading(new Vector2d(62, 32.25), 90)
//                                .addTemporalMarker(() -> {
//                                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
//                                    bot.setWristPosition(wristState.outtaking);
//                               })

                                .resetVelConstraint()
                                //   .strafeRight(3)
                                .build();
                        drive.followTrajectorySequenceAsync(toBackBoard);
                        toboard = true;
                    }
                    if(!drive.isBusy()){
                        currentState = state.leaveBackboard;
                    }




                    break;
                case scoring:
                    if(!drive.isBusy()) {
                        if(!aligned) {
                            if (bot.distanceSensor.getRightEdgeDistance() < 26.75) {
                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - (26 - bot.distanceSensor.getRightEdgeDistance())))


                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(lineUp);
                                aligned = true;
                            }
                            if (bot.distanceSensor.getRightEdgeDistance() > 29.25) {
                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + (bot.distanceSensor.getRightEdgeDistance() - 28)))
                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(lineUp);
                                aligned = true;
                            }

                        }

                            currentState = state.leaveBackboard;

                    }
                case leaveBackboard:
                    if(!drive.isBusy() && !LeaveBackboard) {
                        TrajectorySequence leaveBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - 10, 39.25))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - 10, 60))
                                .build();
                        drive.followTrajectorySequenceAsync(leaveBackboard);
                        LeaveBackboard = true;

                    }
                    if(!drive.isBusy() && !lineUp2){
                        TrajectorySequence lineUpWithTrussWall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.25))


                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(lineUpWithTrussWall);
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 -  bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                       lineUp2 = true;
                    }
                    if(!drive.isBusy()){
                        currentState = state.underTrussFromBackboard;
                    }
                    break;
                case underTrussFromBackboard:
                    if(!drive.isBusy()){
                        TrajectorySequence UnderTrussFromBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.25))


                                .resetVelConstraint()
                                .build();
                    }

                    break;

                case idle:
                    telemetry.addData("Right Edge Distance", bot.distanceSensor.getRightEdgeDistance());
                    telemetry.addLine("Inside Idle State");
                    telemetry.addData("NewPose", drive.getPoseEstimate());
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

            NewVision.StartingPosition startingPos = NewVision.StartingPosition.LEFT;
            visionPortal.resumeStreaming();
            telemetry.addLine("vision portal built");
            telemetry.addData("starting position: ", startingPos);
            startingPos = newVision.getStartingPosition();
            telemetry.addData("called NewVision- returned: ", startingPos);
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

