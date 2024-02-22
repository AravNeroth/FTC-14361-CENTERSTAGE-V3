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
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, underTrussBool = false, stackBool = false, toboard = false,finishBoard = false, lineUp = false, aligned = false, LeaveBackboard = false, lineUp2 = false, findTag = false, UnderTruss = false,LineUp3 = false, UnderTrussToStack = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    double centerTagX = 60.275, centerTagY = 29.4;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, REDSTACK = 7;
    double trussOfset = 0;
    int ID_TAG_OF_INTEREST = -1;
    double tagOffset = 0;
    double yOffset = 0, xOffset = 0;
    double batteryOffset = 0;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

   state currentState = state.tape;

    enum state {
        tape, underTruss,firstTimeBoard, secondTimeBoard, thirdTimeBoard, toStack, idle,leaveStack, park, scoring, leaveBackboard, underTrussFromBackboard, findBackBoardTag, lineUp, localizeAfterStack, finalBackBoardScore
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(37.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
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


                .lineToLinearHeading(new Pose2d(-38.25, 60, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-45.25, 38))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-45.25, 55))
                .lineToLinearHeading(new Pose2d(-38.25, 60, Math.toRadians(180)))
                .waitSeconds(.1)

                .build();
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 90, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-41, 35, Math.toRadians(180)))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToConstantHeading(new Vector2d(-36.75, 35))
                .lineToConstantHeading(new Vector2d(-40, 35))
                .lineToConstantHeading(new Vector2d(-38.25, 60))

//
                .build();


        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());


        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
//         = newVision.getStartingPosition();
        NewVision.StartingPosition startPath = newVision.getStartingPosition();
        switch (newVision.getStartingPosition()) {
            case CENTER:
                yOffset = 1;
                xOffset = 0;
                ID_TAG_OF_INTEREST = -1;
                temporalMarkerTimer.reset();
                drive.followTrajectorySequenceAsync(centerTape);
                telemetry.addLine("CENTER.");
                telemetry.update();

                break;
            case RIGHT:
                ID_TAG_OF_INTEREST = -1;
                temporalMarkerTimer.reset();
                yOffset = -1;

                drive.followTrajectorySequenceAsync(rightTape);
                telemetry.addLine("right.");
                telemetry.update();

                break;
            case LEFT:
                ID_TAG_OF_INTEREST = -1;
                temporalMarkerTimer.reset();
                trussOfset = 1.5;
                tagOffset = 10;
                yOffset = 3;
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

                    break;
                case underTruss:
                    telemetry.addData("Pose Est", drive.getPoseEstimate());
                    if (!drive.isBusy() && !lineUp) {
                        TrajectorySequence lineUpWithTrussWall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.5 + trussOfset))


                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequence(lineUpWithTrussWall);
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 - bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                        lineUp = true;
                    }
                    if(!drive.isBusy()){
                        if(!underTrussBool){
                            TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, 45, DriveConstants.TRACK_WIDTH))
                                    .lineToConstantHeading(new Vector2d(40, drive.getPoseEstimate().getY()- yOffset))

                                    .waitSeconds(.05)
                                    .resetVelConstraint()
                                    .build();
                            drive.followTrajectorySequenceAsync(underTruss);
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 -  bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                            underTrussBool = true;

                        }
                    }

                    if (!drive.isBusy()) {
                        TrajectorySequence toTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 20 + tagOffset))
                                .build();
                        drive.followTrajectorySequenceAsync(toTag);
                        currentState = state.firstTimeBoard;
                    timer.reset();

        }
        break;

                case firstTimeBoard: List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                ID_TAG_OF_INTEREST = -1;
                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                            if (detection.metadata != null) {

                                telemetry.addLine("Inside Metadata If");
                                telemetry.update();

                                //  Check to see if we want to track towards this tag.
                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST) && !finishBoard) {
                                    drive.breakFollowing();
                                    telemetry.addLine("Inside Tag Of Interest If");
                                    telemetry.update();
                                    // Yes, we want to use this tag.



                                    tagFound = true;
                                    tagOfInterest = detection;
                                }

                            }

                            if (tagFound && !findTag) {
                                telemetry.addLine("Inside TagFound If Statement");
                                telemetry.update();
                                timer.reset();

                                temporalMarkerTimer.reset();
                                // final double distanceX = tagOfInterest.center.x;

                                //   tagY = drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.y-3;
                               switch (startPath){
                                   case LEFT:
                                       TrajectorySequence toBackBoardLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                               .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30.5, 45, DriveConstants.TRACK_WIDTH))
                                               .lineToConstantHeading(new Vector2d(40,47.5))
                                               .addTemporalMarker(() -> {
                                                   bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                                   bot.setWristPosition(wristState.outtaking);
                                                   bot.outtakeSlide.setPosition(700);
                                               })
                                               .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y -3.35 + xOffset,47.5))
                                               .addTemporalMarker(() -> {
                                                   bot.setLidPosition(lidState.open);
                                                   bot.outtakeSlide.setPosition(800);
                                               })

                                               .resetVelConstraint()

                                               .build();
                                       drive.followTrajectorySequenceAsync(toBackBoardLeft);
                                       findTag = true;
                                       xOffset = 1;
                                   //    yOffset = -3;
                                       currentState = state.leaveBackboard;
                                       break;
                                   case CENTER:
                                       TrajectorySequence toBackBoard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                               .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30.5, 45, DriveConstants.TRACK_WIDTH))
                                               .lineToConstantHeading(new Vector2d(40,37.25))
                                               .addTemporalMarker(() -> {
                                                   bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                                   bot.setWristPosition(wristState.outtaking);
                                                   bot.outtakeSlide.setPosition(700);
                                               })
                                               .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y -3.35,37.25))
                                               .addTemporalMarker(() -> {
                                                   bot.setLidPosition(lidState.open);
                                                   bot.outtakeSlide.setPosition(800);
                                               })

                                               .resetVelConstraint()

                                               .build();
                                       drive.followTrajectorySequenceAsync(toBackBoard);
                                       findTag = true;
                                       yOffset = 0;
                                       currentState = state.leaveBackboard;

                                       break;
                                   case RIGHT:
                                       TrajectorySequence toBackBoardRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                               .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30.5, 45, DriveConstants.TRACK_WIDTH))
                                               .lineToConstantHeading(new Vector2d(40,33.5))
                                               .addTemporalMarker(() -> {
                                                   bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                                   bot.setWristPosition(wristState.outtaking);
                                                   bot.outtakeSlide.setPosition(700);
                                               })
                                               .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y -3.35,33.5))
                                               .addTemporalMarker(() -> {
                                                   bot.setLidPosition(lidState.open);
                                                   bot.outtakeSlide.setPosition(800);
                                               })

                                               .resetVelConstraint()

                                               .build();
                                       drive.followTrajectorySequenceAsync(toBackBoardRight);
                                       yOffset = 3;
                                       findTag = true;
                                       currentState = state.leaveBackboard;

                                       break;

                               }



                                telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.x);
                                telemetry.addData("Tag ID", tagOfInterest.id);
                                telemetry.addData("FTC Pose y: ", tagOfInterest.ftcPose.y);
                                telemetry.addData("Field Pose ", tagOfInterest.metadata.fieldPosition);
                                telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);

                                telemetry.addLine("Traj Seq Builder ran");
                                telemetry.update();
                            } else {
                                telemetry.addData("Different Tag Found", detection.id);
                                telemetry.addData("Different Tag X", detection.ftcPose.y);
                                telemetry.addData("Different Tag Y", detection.ftcPose.x);
                                telemetry.addData("Bearing", detection.ftcPose.bearing);
                                telemetry.update();
                            }
                        }


                        //    ID_TAG_OF_INTEREST = REDSTACK;


                    } // detect for loop end

                    // if detect not 0 end
//                    else{
//                        tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .lineToConstantHeading(new Vector2d(1, 1))
//                                .build();
//                    }

                    if(!drive.isBusy() && !toboard && !findTag) {
                        switch (startPath){
                            case LEFT:
                                TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.5, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(40,47.5))
                                        .addTemporalMarker(() -> {
                                            bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                            bot.setWristPosition(wristState.outtaking);
                                            bot.outtakeSlide.setPosition(700);
                                        })

                                        .lineToConstantHeading(new Vector2d(50.8,47.5))
                                        .addTemporalMarker(() -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(800);
                                        })
                                        .waitSeconds(.05)
//
                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(toLeftBackboard);
                            yOffset = 5.5;
                                toboard = true;

                                currentState = state.leaveBackboard;
                                break;
                            case CENTER:
                                TrajectorySequence toBackBoard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.5, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(40,37.25))
                                        .addTemporalMarker(() -> {
                                            bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                            bot.setWristPosition(wristState.outtaking);
                                            bot.outtakeSlide.setPosition(700);
                                        })

                                        .lineToConstantHeading(new Vector2d(51,37.25))
                                        .addTemporalMarker(() -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(800);
                                        })
                                        .waitSeconds(.05)
//
                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(toBackBoard);

                                toboard = true;
                                yOffset = -2.75;

                                currentState = state.leaveBackboard;
                                break;
                            case RIGHT:
                                TrajectorySequence toRightBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.5, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(40,33.5))
                                        .addTemporalMarker(() -> {
                                            bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                            bot.setWristPosition(wristState.outtaking);
                                            bot.outtakeSlide.setPosition(700);
                                        })

                                        .lineToConstantHeading(new Vector2d(50.8,33.5))
                                        .addTemporalMarker(() -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(800);
                                        })
                                        .waitSeconds(.05)
//
                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(toRightBackboard);
                                yOffset = 2.5;

                                toboard = true;
                                currentState = state.leaveBackboard;
                                break;
                        }

                    }
                    if(!drive.isBusy() && (toboard || findTag)){
                        currentState = state.leaveBackboard;
                    }




                    break;
                case scoring:
                    if(!drive.isBusy()) {
                        if(!aligned) {
                            if (bot.distanceSensor.getRightEdgeDistance() < 26.75) {
                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - (26 - bot.distanceSensor.getRightEdgeDistance())))


                                        .resetVelConstraint()
                                        //   .strafeRight(3)
                                        .build();
                                drive.followTrajectorySequenceAsync(lineUp);
                                aligned = true;
                            }
                            if (bot.distanceSensor.getRightEdgeDistance() > 29.25) {
                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
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
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - 20, drive.getPoseEstimate().getY()))
                                .addDisplacementMarker(4,() -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                        })
                                .addDisplacementMarker(8,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - 20, 61.25 + yOffset))
                                .build();
                        drive.followTrajectorySequenceAsync(leaveBackboard);
                        LeaveBackboard = true;
                    currentState = state.underTrussFromBackboard;
                    }


                    break;
                case underTrussFromBackboard:
                    if(!drive.isBusy() && !UnderTrussToStack){
                        switch (startPath){
                            case RIGHT:
                                yOffset = -1.5;
                                break;
                            case CENTER:
                                yOffset = 3.25;
                                break;
                            case LEFT:
                                yOffset = -2;
                                break;
                        }
                        telemetry.addData("Under Truss" ,drive.getPoseEstimate().getY());
                        telemetry.update();
                        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(42.5, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(-40, drive.getPoseEstimate().getY() + yOffset))

                                .build();
                        drive.followTrajectorySequenceAsync(underTruss);
                        UnderTrussToStack = true;
                    }
                    if(!drive.isBusy() && UnderTrussToStack) {
                        telemetry.addData("LINE UP again" ,drive.getPoseEstimate().getY());
                        telemetry.update();
                        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.25))

                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(lineUp);
                        switch (startPath)
                        {
                            case LEFT:
                                yOffset = 2.25;
                                xOffset = 1.5;
                                break;
                            case CENTER:
                                yOffset = 0;
                                break;
                            case RIGHT:
                                xOffset = .25;
                                yOffset = .5;
                                break;
                        }
                        currentState = state.toStack;
                    }


                    break;
                case toStack:
                    if(!drive.isBusy()) {
                        TrajectorySequence toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                             //   .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), 43))
                                .addDisplacementMarker(4,() -> {
                                   bot.setLinkagePosition(linkageState.HIGH);
                                })

                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX()-18.1 - xOffset, 43 - yOffset, Math.toRadians(130)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                              //  .turn(Math.toRadians(-50))

                                .turn(Math.toRadians(100))
                                .addTemporalMarker(() -> {
                                    bot.setLinkagePosition(linkageState.LOW);
                                  bot.setActiveIntakePosition(activeIntakeState.active);
                                })
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX()-19.5 - xOffset, 35.5- yOffset, Math.toRadians(180)))
                             //   .turn(Math.toRadians(-50))
                                .forward(2)
                                //.lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX()-21.5,43))

                                .waitSeconds(.25)
                                .addTemporalMarker(() -> {
                                    bot.setArmPosition(armState.init, armExtensionState.extending);

                                })

                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() - xOffset, 61.25, Math.toRadians(180)))
                                .addDisplacementMarker(40,() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                })

                             //   .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), 61.25))
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                                })
                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        aligned = false;
                        currentState = state.localizeAfterStack;
                    }
break;
                case localizeAfterStack:
                    if(!drive.isBusy() && !aligned){
                        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + bot.distanceSensor.getRightEdgeDistance() - 3.25))

                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(lineUp);
                        aligned = true;
                    }
                    if(!drive.isBusy() && aligned){
                        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(40, drive.getPoseEstimate().getY()))
                                .addTemporalMarker(() -> {
                                    bot.setArmPosition(armState.init, armExtensionState.extending);
                                   bot.setLidPosition(lidState.close);
                                })

                                .build();
                        drive.followTrajectorySequenceAsync(underTruss);
                        currentState = state.finalBackBoardScore;
                    }
                    break;
                case finalBackBoardScore:
                    if(!drive.isBusy()){
                        TrajectorySequence Score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35 , 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(40, 44))
                                .addDisplacementMarker(() -> {
                                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.outtaking);
                                })
                                .addDisplacementMarker(15,() -> {

                                    bot.outtakeSlide.setPosition(800);
                                })
                                .lineToConstantHeading(new Vector2d(51, 44))
                                .addTemporalMarker(() -> {
                                    bot.setLidPosition(lidState.open);
                                    bot.outtakeSlide.setPosition(900);
                                })
                                .waitSeconds(.1)
                                .lineToConstantHeading(new Vector2d(45, 44))
                                .addTemporalMarker(() -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                   bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                                })
                                .lineToConstantHeading(new Vector2d(45, 61.5))
                                .lineToConstantHeading(new Vector2d(50, 61.5))
                                .build();
                        drive.followTrajectorySequenceAsync(Score);
                        currentState = state.idle;
                    }

                    break;
                case idle:
                    telemetry.addData("Right Edge Distance", bot.distanceSensor.getRightEdgeDistance());
                    telemetry.addLine("Inside Idle State");
                    telemetry.addData("NewPose", drive.getPoseEstimate());
                    telemetry.addData("Y" ,drive.getPoseEstimate().getY());
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
    private void initAprilTag2ndCam() {
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
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
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

