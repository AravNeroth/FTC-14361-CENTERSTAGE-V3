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

@Autonomous(name = "Close Blue April Tag Stack ", group = "goobTest")

public class CloseBlueAprilTagStack extends LinearOpMode {

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
    Pose2d start = new Pose2d(11.5, 62.75, Math.toRadians(0));
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
    TrajectorySequence pickUpStack = null;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    state currentState = state.idle;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack, idle, park, leaveBoard, underGateToStack, leaveStack, stackTurn, scoreStack, stackTurnOtherWay, stackOtherWay
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

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
        switch (newVision.getStartingPosition()){
            case CENTER:
                ID_TAG_OF_INTEREST = MIDDLE;
                temporalMarkerTimer.reset();
             //   drive.followTrajectorySequenceAsync(centerTape);
               // telemetry.addLine("CENTER.");
                telemetry.update();

                break;
            case RIGHT:
                ID_TAG_OF_INTEREST = RIGHT;
                temporalMarkerTimer.reset();

               // drive.followTrajectorySequenceAsync(rightTape);
            //    telemetry.addLine("right.");
                telemetry.update();

                break;
            case LEFT:
                ID_TAG_OF_INTEREST = LEFT;
                temporalMarkerTimer.reset();

            //    drive.followTrajectorySequenceAsync(leftTape);
             //   telemetry.addLine("left.");
                telemetry.update();

                break;
        }
        currentState = state.thirdTimeBoard;
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
                        if(ID_TAG_OF_INTEREST == MIDDLE){
                            drive.followTrajectorySequenceAsync(goToCenterAprilTag);
                            offset = 3;
                        }
                        else if(ID_TAG_OF_INTEREST == LEFT){
                            drive.followTrajectorySequenceAsync(goToLeftAprilTag);
                            offset = 5.75;
                        } else if (ID_TAG_OF_INTEREST == RIGHT) {
                            drive.followTrajectorySequenceAsync(goToRightAprilTag);
                            offset = 2.5;
                        }

                        currentState = state.firstTimeBoard;
                        temporalMarkerTimer.reset();
                        timer.reset();
                    }
//
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case firstTimeBoard:
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

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


//                                    if(drive.getPoseEstimate().getY() - detection.metadata.fieldPosition.get(1) > 0){
//                                        tagY = drive.getPoseEstimate().getY() - (-detection.ftcPose.x);
//                                    }
//                                    else {
//                                        tagY = drive.getPoseEstimate().getY() + (-detection.ftcPose.x);
//                                    }
                                    tagY = drive.getPoseEstimate().getY() + (detection.ftcPose.x);
                                    tagFound = true;
                                    tagOfInterest = detection;
                                }
//                                else if(!randomTag){
//                                    TrajectorySequence turnToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                            .turn(detection.ftcPose.elevation)
//                                            .build();
//                                    drive.followTrajectorySequenceAsync(turnToAprilTag);
//                                    randomTag = true;
//                                }
                            }

                            if (tagFound) {
                                telemetry.addLine("Inside TagFound If Statement");
                                telemetry.update();
                                timer.reset();

                                temporalMarkerTimer.reset();
                                // final double distanceX = tagOfInterest.center.x;

                                //   tagY = drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.y-3;
                                tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y -3, tagY - offset))
                                        .addDisplacementMarker( 1, () -> {
                                            bot.outtakeSlide.setPosition(525);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(700);
                                        })
                                        .waitSeconds(.15)
                                        //   .lineToConstantHeading(new Vector2d(50, tagY - 8))

                                        .build();



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



                    if (tagFound) {
                        telemetry.addData("Field Pose ", tagOfInterest.metadata.fieldPosition);
                        telemetry.addData("Tag ID", tagOfInterest.id);
                        telemetry.update();
                        if(!finishBoard){
                            drive.followTrajectorySequenceAsync(tag);
                            finishBoard = true;
                        }
                        if(!drive.isBusy()){
                            drive.setPoseEstimate(new Pose2d(tagOfInterest.metadata.fieldPosition.get(0)-8,tagOfInterest.metadata.fieldPosition.get(1), Math.toRadians(180)));
                            telemetry.addLine("Reset Pose");

                            telemetry.addData("New Pose", drive.getPoseEstimate());
                            currentState = state.leaveBoard;
                        }

                    }

                    //  currentState = state.firstTimeBoard;



                    break;
                case leaveBoard:
                    if(!drive.isBusy()){
                        TrajectorySequence leave = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.1)
                                .lineToConstantHeading(new Vector2d(42, drive.getPoseEstimate().getY()))
                                .waitSeconds(.2)
                                .lineToConstantHeading(new Vector2d(42, 12.25))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.1)
                                //.splineTo(new Vector2d(42, 10), Math.toRadians(180))
                                .addDisplacementMarker(5,  () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(10,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

                                })
                                .build();
                        drive.followTrajectorySequenceAsync(leave);
                        currentState = state.underGateToStack;
                    }
                    break;
                case underGateToStack:
                    if(!drive.isBusy()){
                        TrajectorySequence underGate = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(47.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(-55, 11.75))

                                .build();
                        drive.followTrajectorySequenceAsync(underGate);

                        currentState = state.stackTurn;
                    }

                    break;
                case stackTurn:
                    if(!drive.isBusy()){
                        if(bot.distanceSensor.getBotsFrontDistance() < 50){
                            drive.setPoseEstimate(new Pose2d(-72 + bot.distanceSensor.getBotsFrontDistance() + 8, drive.getPoseEstimate().getY(), Math.toRadians(180)));
                            pickUpStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-61.25, 11.65, Math.toRadians(210)))
                                    .addTemporalMarker(() -> {
                                        //     bot.setActiveIntakePosition(activeIntakeState.active);
                                        bot.setLinkagePosition(linkageState.LOW);
                                    })
                                    .forward(bot.distanceSensor.getBotsFrontDistance() - 4)
                                    .build();
                        }
                        else{
                            pickUpStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-61.25, 11.65, Math.toRadians(210)))
                                    .addTemporalMarker(() -> {
                                        //     bot.setActiveIntakePosition(activeIntakeState.active);
                                        bot.setLinkagePosition(linkageState.LOW);
                                    })
                                    .forward(1)
                                    .build();
                        }


                        drive.followTrajectorySequenceAsync(pickUpStack);
                        currentState = state.stack;
                    }
                    break;
                case stackTurnOtherWay:
                    if(!drive.isBusy()){
                        if(bot.distanceSensor.getBotsFrontDistance() < 50){
                            drive.setPoseEstimate(new Pose2d(-72 + bot.distanceSensor.getBotsFrontDistance() + 8, drive.getPoseEstimate().getY(), Math.toRadians(180)));
                            pickUpStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-61.25, 18, Math.toRadians(180)))
                                    .addTemporalMarker(() -> {
                                        //     bot.setActiveIntakePosition(activeIntakeState.active);
                                        bot.setLinkagePosition(linkageState.HIGH);
                                    })
                                    .forward(bot.distanceSensor.getBotsFrontDistance() - 4)
                                    .build();
                        }
                        else{
                            pickUpStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-61.25, 18, Math.toRadians(210)))
                                    .addTemporalMarker(() -> {
                                        //     bot.setActiveIntakePosition(activeIntakeState.active);
                                        bot.setLinkagePosition(linkageState.HIGH);
                                    })
                                    .forward(1)
                                    .build();
                        }


                        drive.followTrajectorySequenceAsync(pickUpStack);
                        currentState = state.stack;
                    }
                    break;
                case stackOtherWay:
                    if(!drive.isBusy()){

                        TrajectorySequence pickUpStack1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                //.lineToLinearHeading(new Pose2d(-63, 15, Math.toRadians(180)))
                                .strafeLeft(5)
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.active);
                                    // bot.setLinkagePosition(linkageState.HIGH);
                                })
                                .waitSeconds(.5)
                                .back(.35)
                                .strafeRight(2)
                                .waitSeconds(.1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(10)
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                                    bot.setArmPosition(armState.init, armExtensionState.extending);

                                })

                                .build();
                        drive.followTrajectorySequenceAsync(pickUpStack1);
                        currentState = state.leaveStack;
                    }
                    break;
                case stack:
                    if(!drive.isBusy()){

                        TrajectorySequence pickUpStack1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .turn(Math.toRadians(-70))
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.active);
                                    // bot.setLinkagePosition(linkageState.HIGH);
                                })
                                .lineToLinearHeading(new Pose2d(-63, 15, Math.toRadians(180)))

                                .waitSeconds(.5)
                                .back(.35)
                                .strafeRight(2)
                                .waitSeconds(.1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(10)
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                                    bot.setArmPosition(armState.init, armExtensionState.extending);

                                })

                                .build();
                        drive.followTrajectorySequenceAsync(pickUpStack1);
                        currentState = state.leaveStack;
                    }
                    break;
                case leaveStack:
                    if(!drive.isBusy()){
                        if(bot.distanceSensor.getBotsRightCenterDistance() < 58) {
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 - bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                        }
                        TrajectorySequence underGate = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), 13.75, Math.toRadians(180)))
                                .addTemporalMarker(.1,() -> {
                                    bot.setLidPosition(lidState.close);
                                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                })
                                .addTemporalMarker(.6,() -> {
                                    bot.setLinkagePosition(linkageState.HIGH);
                                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(47.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(38, 13.75))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(38, 34))
                                .addTemporalMarker(() -> {
                                    bot.outtakeSlide.setPosition(700);
                                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.outtaking);
                                })
                                .waitSeconds(.1)

                                // .lineToConstantHeading(new Vector2d(45, 28))

                                .build();
                        drive.followTrajectorySequenceAsync(underGate);

                        currentState = state.scoreStack;
                    }
                    break;
                case scoreStack:
                    if(!drive.isBusy()) {
                        if(bot.distanceSensor.getBotsRightCenterDistance() < 58) {
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 72 - bot.distanceSensor.getBotsRightCenterDistance(), Math.toRadians(180)));
                        }
                        TrajectorySequence scoreSt = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                //    .lineToConstantHeading(new Vector2d(51.5, bot.distanceSensor.getBotsRightCenterDistance()))
                                .lineToConstantHeading(new Vector2d(52.3, 31.75))
                                .addTemporalMarker(() -> {
                                    bot.setLidPosition(lidState.open);
                                    bot.outtakeSlide.setPosition(950);
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(scoreSt);
                        currentState = state.park;
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

                                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(270)))

//

                                .lineToConstantHeading(new Vector2d(51, 12))
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
                        drive.followTrajectorySequenceAsync(parkNextToBackboard);
                        currentState = state.idle;
                    }
                    break;
                case thirdTimeBoard:
                    if(!drive.isBusy()){
                        double leftStrafe = bot.ultrasonicSensor.avgOf(5);
                        double offset = 0;
                        if(leftStrafe > 48.5){
                            //50
                            offset = 42;
                        }
                        else{
                            offset = 46;
                        }
                        TrajectorySequence parkInCorner = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .strafeLeft(leftStrafe - offset)
                                .build();
                        drive.followTrajectorySequenceAsync(parkInCorner);
                        currentState = state.idle;
                        }
                    break;
                case idle:
                    telemetry.addLine("Inside Idle State");


                    telemetry.addData("pose est ", drive.getPoseEstimate());

                    telemetry.addData("Left Center Dis", bot.distanceSensor.getBotsLeftCenterDistance());
                    telemetry.addData("Left Edge Distance", bot.distanceSensor.getBotsLeftEdgeDistance());
                    telemetry.addData("Pose Est", drive.getPoseEstimate());
                    telemetry.addData("Y Value", drive.getPoseEstimate().getY());




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
