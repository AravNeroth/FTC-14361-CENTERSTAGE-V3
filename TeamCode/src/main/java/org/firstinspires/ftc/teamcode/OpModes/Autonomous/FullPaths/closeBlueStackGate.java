package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths;

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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing.LongBlueDistanceSensor;
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

@Autonomous(name = "CloseBlueStackGate ", group = "goobTest")
public class closeBlueStackGate extends LinearOpMode {

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
    boolean cameraOn = false, aprilTagOn = false, initCam = false, toboard = false, finishBoard = false, stack1 = false, aligned = false, LeaveBackboard = false, findTag = false, approachingStack = false;
    double tagY = 0;
    double detectYPos = 0, detectYNeg = 0;
    double tagOffset = 0;
    double yOffset = 0, xOffset = 0;
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, BLUESTACK = 9;
    int ID_TAG_OF_INTEREST = 4;
    boolean tagFound = false;
    double offset = 3;

    state currentState = state.tape;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack1, idle, park, leaveBackboard, scoring
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
                .build();

        TrajectorySequence goToLeftAprilTag = drive.trajectorySequenceBuilder(leftTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(25, 34))
                .build();

        TrajectorySequence goToRightAprilTag = drive.trajectorySequenceBuilder(rightTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 90, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(30, 24))
                .build();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());

        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();

        if (isStopRequested()) return;
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

                    break;

                case firstTimeBoard:
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
                                        currentState = closeBlueStackGate.state.leaveBackboard;
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
                                        currentState = closeBlueStackGate.state.leaveBackboard;

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
                                        currentState = closeBlueStackGate.state.leaveBackboard;

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
                    }

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
                                        .resetVelConstraint()
                                        .build();
                                drive.followTrajectorySequenceAsync(toLeftBackboard);
                                yOffset = 5.5;
                                toboard = true;

                                currentState = closeBlueStackGate.state.leaveBackboard;
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
                                        .resetVelConstraint()
                                        .build();

                                drive.followTrajectorySequenceAsync(toBackBoard);

                                toboard = true;
                                yOffset = -2.75;

                                currentState = closeBlueStackGate.state.leaveBackboard;
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
                                        .resetVelConstraint()
                                        .build();
                                drive.followTrajectorySequenceAsync(toRightBackboard);
                                yOffset = 2.5;

                                toboard = true;
                                currentState = closeBlueStackGate.state.leaveBackboard;
                                break;
                        }
                    }

                    if(!drive.isBusy() && (toboard || findTag)){
                        currentState = closeBlueStackGate.state.leaveBackboard;
                    }

                    break;

//                case scoring:
//                    if(!drive.isBusy()) {
//                        if(!aligned) {
//                            if (bot.distanceSensor.getRightEdgeDistance() < 26.75) {
//                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
//                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - (26 - bot.distanceSensor.getRightEdgeDistance())))
//
//                                        .resetVelConstraint()
//                                        .build();
//                                drive.followTrajectorySequenceAsync(lineUp);
//                                aligned = true;
//                            }
//                            if (bot.distanceSensor.getRightEdgeDistance() > 29.25) {
//                                TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + (bot.distanceSensor.getRightEdgeDistance() - 28)))
//                                        .resetVelConstraint()
//                                        .build();
//                                drive.followTrajectorySequenceAsync(lineUp);
//                                aligned = true;
//                            }
//                        }
//                        currentState = closeBlueStackGate.state.leaveBackboard;
//                    }

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
                                .lineToConstantHeading(new Vector2d(47, 12))
                                .build();
                        drive.followTrajectorySequenceAsync(leaveBackboard);
                        LeaveBackboard = true;
                        currentState = closeBlueStackGate.state.stack1;
                    }

                case stack1:
                    if(!drive.isBusy() && !stack1) {

                        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 35, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(-58,11))
                                .addDisplacementMarker(50,() -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                    bot.setLinkagePosition(linkageState.supaRus);
                                })
                                .resetVelConstraint()
                                .build();

                        drive.followTrajectorySequence(stack1);

                        TrajectorySequence intaking = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 35, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(-61,11))
                                .addDisplacementMarker(2,() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.active);
                                })
                                .waitSeconds(1.5)
                                .lineToConstantHeading(new Vector2d(47,11))
                                .addDisplacementMarker(40,() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                    bot.setWristPosition(wristState.init);
                                    bot.setLidPosition(lidState.close);
                                })
                                .resetVelConstraint()
                                .build();

                        if(!drive.isBusy() && !approachingStack)
                        {
                            drive.followTrajectorySequenceAsync(intaking);
                            bot.setLinkagePosition(linkageState.LOW);
                            bot.setLinkagePosition(linkageState.supaRus);
                            bot.setLinkagePosition(linkageState.LOW);
                            bot.setLinkagePosition(linkageState.supaRus);
                            approachingStack = true;
                        }
                    }
                    stack1 = true;
                    currentState = closeBlueStackGate.state.secondTimeBoard;
                    break;

                case secondTimeBoard:
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
                                .build();
                        drive.followTrajectorySequenceAsync(Score);
                        currentState = closeBlueStackGate.state.park;
                    }

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
                                .lineToLinearHeading(new Pose2d(50, 61.5, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(52, 55))
                                .build();
                        drive.followTrajectorySequenceAsync(parkInCorner);
                        currentState = state.idle;
                    }
                case idle:
                    telemetry.addLine("Inside Idle State");
                    telemetry.addData("Tag ID", tagOfInterest.id);
                    telemetry.addData("pose est ", drive.getPoseEstimate());
                    telemetry.addData("Tag y", tagY);
                    telemetry.addData("double y + ", detectYPos);
                    telemetry.addData("double y -", detectYNeg);
                    telemetry.addData("tag ", tagOfInterest.metadata.fieldPosition.get(1)+5);

                    telemetry.update();
                    break;
            }
            drive.update();
        }

    }

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
                    .build();

            visionPortal.resumeStreaming();
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
