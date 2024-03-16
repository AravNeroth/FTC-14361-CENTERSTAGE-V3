package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Stack;

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

@Autonomous(name = "BETA RED", group = "Stack")
public class LongRedUltrasonicUnderTruss extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    boolean timerReset = false;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    NewVision newVision;


    String webcamName;
    Robot bot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime temporalMarkerTimer = new ElapsedTime();
    Pose2d start = new Pose2d(-36.5, -62.75, Math.toRadians(270));
    SampleMecanumDrive drive;
    String selection;
    OpenCvCamera camera;
    double offset = 0;
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
    TrajectorySequence toStack = null,    toStackTraj = null;;


    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;
    int lineUpCount = 0, underTrussCount = 0;
    double underTrussOffset = 0, stackDistanceOffset = 0;
    double lineUpOffset = 0;
    state currentState = state.tape;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, stack, idle, park, underGate, lineUp, underTruss, toAprilTags, leaveBoard, setPoseEstimate, underTrussToStack, toStack, boardFromStack, forwardStack, leaveStack
    }
double headingOffset = 0;
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToConstantHeading(new Vector2d(-40, -35))
                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-40, -40))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //    .lineToConstantHeading(new Vector2d(-38.25, -40))
                .lineToLinearHeading(new Pose2d(-40, -60, Math.toRadians(180)))
                //.turn(Math.toRadians(1.25))

                .resetVelConstraint()
                .build();
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-47,-52))

                //Close lid
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Push to tape
                .lineToConstantHeading(new Vector2d(-48.25, -44))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-48.25, -50))
                .lineToLinearHeading(new Pose2d(-48.25,-60, Math.toRadians(180)))
                //Move to center
                //    .lineToConstantHeading(new Vector2d(-39,-50))
                // .setConstraints(40, 40)
                //  .lineToConstantHeading(new Vector2d(-39,-60))
                .turn(Math.toRadians(-90))

                // .lineToLinearHeading(new Pose2d(-37.5,-11, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(25,-11))
                .addTemporalMarker(() -> {
                 //   bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                  //  bot.setWristPosition(wristState.outtaking);
                })
                //.turn(Math.toRadians(3))

                .build();
        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-41, -35, Math.toRadians(225)))

                .addTemporalMarker(.05, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15, () -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(-34.75, -35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-42, -35))
                .lineToConstantHeading(new Vector2d(-42, -60))
                //   .lineToConstantHeading(new Vector2d(40, -11))

                .build();



        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());



        telemetry.update();

        TrajectorySequence tag = null;
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
                headingOffset = 6.5;

                break;
            case RIGHT:
                ID_TAG_OF_INTEREST = MIDDLE;
                temporalMarkerTimer.reset();

                drive.followTrajectorySequenceAsync(centerTape);

                telemetry.addLine("right.");
                telemetry.update();

                break;
            case LEFT:
                ID_TAG_OF_INTEREST = MIDDLE;
                temporalMarkerTimer.reset();
                headingOffset = 2;
                drive.followTrajectorySequenceAsync(centerTape);
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

                        currentState = state.setPoseEstimate;
                        temporalMarkerTimer.reset();
                        timer.reset();
                    }
//
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case setPoseEstimate:
                    if(!drive.isBusy()){
                        //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + bot.distanceSensor.getBotsLeftCenterDistance(), Math.toRadians(180)));
                        currentState = state.lineUp;
                    }
                    break;

                case lineUp:
                    if(!drive.isBusy()){

                        double poseEstSum = 0;
                        double edgeSum = 0;
                        int count = 0;

                        for (int x = 0; x < 10; x++) {
                            double distanceCent = bot.ultrasonicSensor.getLeftDistanceCenter();
                            double distanceEdge = distanceCent - 7.75;
                            if (distanceCent < 80) {
                                poseEstSum += distanceCent;
                                edgeSum += distanceEdge;

                                count++;

                            }
                        }
                        if (count != 0) {
                            poseEstSum /= count;
                            edgeSum /= count;
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + poseEstSum, drive.getPoseEstimate().getHeading()));


                        }


                        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, 45, DriveConstants.TRACK_WIDTH))
                              //  .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()))
                                .strafeLeft(poseEstSum - 16 - lineUpOffset)
                                .resetVelConstraint()
                                .build();




                        if(lineUpCount == 0){
                            //after tape
                            lineUpCount++;

                            drive.followTrajectorySequenceAsync(lineUp);

                            currentState = state.underTruss;

                        } else if (lineUpCount == 1) {
                            //after u go under truss
                            lineUpCount++;
                            //  drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + poseEstSum, Math.toRadians(180)));
                            drive.followTrajectorySequenceAsync(lineUp);

                            currentState = state.toAprilTags;

                            lineUpOffset = 2;
                        } else if (lineUpCount == 2) {
                            //after u score
                            lineUpOffset = 0;
                            lineUpCount++;
                         //   drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + poseEstSum, Math.toRadians(180)));
                            drive.followTrajectorySequenceAsync(lineUp);
                            currentState = state.underTrussToStack;
                        }
                        else if (lineUpCount == 3) {
                            //after u go under the truss to the stack
                            lineUpOffset = 3;

                            lineUpCount++;
                            currentState = state.toStack;
                        } else if (lineUpCount ==4) {
                            //leaving stack back under truss
                            lineUpCount++;
                            underTrussOffset = -1.25;
                            drive.followTrajectorySequenceAsync(lineUp);
                            currentState = state.underTruss;

                        } else if (lineUpCount == 5) {
                            lineUpCount++;
                            stackDistanceOffset = 0;
                            currentState = state.boardFromStack;
                        }
                    }
                    break;
                case underTruss:
                    if(!drive.isBusy()){
                        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(30 ,drive.getPoseEstimate().getY() + 5 + underTrussOffset))
                                .addTemporalMarker(() -> {
                                   bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.outtaking);
                                })
                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(underTruss);
                        currentState = state.setPoseEstimate;
                    }
                    break;
                case toAprilTags:
                    if(!drive.isBusy()) {
                        if (ID_TAG_OF_INTEREST == MIDDLE) {
                            TrajectorySequence goToCenterAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .lineToLinearHeading(new Pose2d(30, drive.getPoseEstimate().getY() + 25, Math.toRadians(181)))


                                    //   .strafeRight(3)
                                    .build();
                            drive.followTrajectorySequenceAsync(goToCenterAprilTag);
                            offset = 2.25;
                            currentState = state.firstTimeBoard;

                        } else if (ID_TAG_OF_INTEREST == LEFT) {
                            TrajectorySequence goToLeftAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .lineToConstantHeading(new Vector2d(35, drive.getPoseEstimate().getY() + 40))


                                    //   .strafeRight(3)
                                    .build();
                            drive.followTrajectorySequenceAsync(goToLeftAprilTag);
                            // 1.2
                            offset = (5);
                            currentState = state.firstTimeBoard;

                        } else if (ID_TAG_OF_INTEREST == RIGHT) {
                            TrajectorySequence goToRightAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .lineToConstantHeading(new Vector2d(35, drive.getPoseEstimate().getY() + 30))


                                    //   .strafeRight(3)
                                    .build();
                            drive.followTrajectorySequenceAsync(goToRightAprilTag);
                            offset = 5.25;
                            currentState = state.firstTimeBoard;
                        }

                    }
                    break;
                case firstTimeBoard:
                    if(!timerReset){
                        timer.reset();
                    }
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

                                    tagY = drive.getPoseEstimate().getY() - (detection.ftcPose.x);
                                    tagFound = true;
                                    tagOfInterest = detection;
                                }

                            }

                            if (tagFound) {
                                telemetry.addLine("Inside TagFound If Statement");
                                telemetry.update();
                                timer.reset();

                                temporalMarkerTimer.reset();
                                // final double distanceX = tagOfInterest.center.x;

                                //   tagY = drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.y-3;
                                tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y - 2.75, tagY - offset))
                                        .addDisplacementMarker( .5, () -> {
                                            bot.outtakeSlide.setPosition(700);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.lid.setLidPosition(lidState.open);
                                           bot.outtakeSlide.setPosition(800);
                                        })
                                        // .waitSeconds(.15)
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

                    } // detect for loop end



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
                    else if(timer.seconds() > 5){
                        if(!drive.isBusy()){
                            switch (startPath){
                                case LEFT:

                                    break;
                                case CENTER:
                                    TrajectorySequence score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                                            .lineToConstantHeading(new Vector2d(52.1, -35))
                                            .addDisplacementMarker( .5, () -> {
                                                bot.outtakeSlide.setPosition(700);
                                            })
                                            .addTemporalMarker( () -> {
                                                bot.lid.setLidPosition(lidState.open);
                                                bot.outtakeSlide.setPosition(800);
                                            })
                                            // .waitSeconds(.15)
                                            //   .lineToConstantHeading(new Vector2d(50, tagY - 8))

                                            .build();
                                    drive.followTrajectorySequenceAsync(score);
                                    currentState = state.leaveBoard;

                                    break;
                                case RIGHT:
                                    break;
                            }
                        }
                    }

                    //  currentState = state.firstTimeBoard;



                    break;
                case leaveBoard:
                    if(!drive.isBusy()) {
                        double poseEstSum = 0;
                        double edgeSum = 0;
                        int count = 0;

                        for (int x = 0; x < 10; x++) {
                            double distanceCent = bot.ultrasonicSensor.getLeftDistanceCenter();
                            double distanceEdge = distanceCent - 7.75;
                            if (distanceCent < 80) {
                                poseEstSum += distanceCent;
                                edgeSum += distanceEdge;

                                count++;

                            }
                        }
                        if (count != 0) {
                            poseEstSum /= count;
                            edgeSum /= count;
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + poseEstSum, Math.toRadians(180)));
                        }
                        TrajectorySequence leaveBackBoard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            //    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX()-10, drive.getPoseEstimate().getY()))
                                .addDisplacementMarker(5, () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(10, () -> {
                                    bot.outtakeSlide.setOuttakeSlidePosition(extensionState.extending, outtakeSlidesState.STATION);
                                })
                                .waitSeconds(.1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX()-30, -57, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequenceAsync(leaveBackBoard);
                        currentState = state.setPoseEstimate;
                    }
                    break;
                case underTrussToStack:
                    if(!drive.isBusy()){
                        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, 45, DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(-40, drive.getPoseEstimate().getY() -1 + underTrussOffset, Math.toRadians(180 + headingOffset)))
                                .resetVelConstraint()
                                .build();

                        drive.followTrajectorySequenceAsync(underTruss);
                        currentState = state.setPoseEstimate;



                    }
                    break;
                case toStack:
                    if(!drive.isBusy()) {
                        headingOffset = 0;
                        double sideSum = 0;
                        int count = 0;

                        for (int x = 0; x < 10; x++) {
                            double distance = bot.ultrasonicSensor.getLeftDistanceCenter();
                            if (distance < 80) {
                                sideSum += distance;
                                count++;

                            }
                        }
                        if (count != 0) {
                            sideSum /= count;
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + sideSum, drive.getPoseEstimate().getHeading()));
                        }
                        double frontSum = 0;
                        int frontCount = 0;

                        for (int x = 0; x < 5; x++) {
                            double distance = bot.distanceSensor.getBotsFrontDistance();
                            if (distance < 58) {
                                frontSum += distance;
                                frontCount++;

                            }
                        }
                        if (frontCount != 0) {
                            frontSum /= frontCount;
                           toStackTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .addDisplacementMarker(1, () -> {
                                      //  bot.setLinkagePosition(linkageState.HIGH);
                                        bot.linkage.setLinkageCustomPosition(.86);
                                    })
                                    .strafeRight(30 - sideSum)
                                    .forward(frontSum -9.2)
                                    //  .lineToLinearHeading(new Pose2d(drive, drive.getPoseEstimate().getY(), Math.toRadians(230)))
                                   // .waitSeconds(.1)
                                   .strafeRight(11.5)
                                   .back(.15)
                                   .strafeLeft(4.5)
                                //   .back(.2)
                                  // .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + .2, drive.getPoseEstimate().getY() - 3, Math.toRadians(180)))
                                   .addTemporalMarker(() -> {
                                       bot.setLinkagePosition(linkageState.LOW);
                                       bot.setActiveIntakePosition(activeIntakeState.active);
                                   })
                                   .waitSeconds(.25)
                                   .strafeRight(3)
                                 //   .turn(Math.toRadians(-100))
//                                    .addTemporalMarker(() -> {
//                                        bot.setLinkagePosition(linkageState.LOW);
//                                        bot.setActiveIntakePosition(activeIntakeState.active);
//                                    })
                                  .waitSeconds(.25)

                                    .build();
                        }
                        else{
//                           toStackTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                                    .addDisplacementMarker(4, () -> {
//                                        bot.setLinkagePosition(linkageState.HIGH);
//                                    })
//                                    .strafeRight(25 - sideSum)
//                                    .forward(10)
//                                    //   .lineToLinearHeading(new Pose2d(-56, drive.getPoseEstimate().getY(), Math.toRadians(230)))
//                                    .waitSeconds(.1)
//                                   .strafeRight(10)
//                                   // .turn(Math.toRadians(-100))
//                                    .addTemporalMarker(() -> {
//                                        bot.setLinkagePosition(linkageState.LOW);
//                                        bot.setActiveIntakePosition(activeIntakeState.active);
//                                    })
//
//                                    .build();
                        }
                        drive.followTrajectorySequenceAsync(toStackTraj);
                        //  underTrussOffset = 3;
                        stackDistanceOffset = 0;
                        currentState = state.leaveStack;
                    }

                    break;
                case leaveStack:
                    if(!drive.isBusy()){
                        double sideSum = 0;
                        int count = 0;

                        for (int x = 0; x < 10; x++) {
                            double distance = bot.ultrasonicSensor.getLeftDistanceCenter();
                            if (distance < 80) {
                                sideSum += distance;
                                count++;

                            }
                        }
                        if (count != 0) {
                            sideSum /= count;
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -72 + sideSum, drive.getPoseEstimate().getHeading()));
                        }

                        TrajectorySequence leave = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                              //  .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(5)
                                .addTemporalMarker(.15,() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.active);

                                })
                                .addTemporalMarker(.25,() -> {
                                        bot.setArmPosition(armState.init, armExtensionState.extending);
                                    })
                                .addTemporalMarker(.4,() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                    bot.setLidPosition(lidState.close);
                                })
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + 20, drive.getPoseEstimate().getY() - 15, Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                                  //  bot.setLidPosition(lidState.close);
                                })



                                .build();
                        drive.followTrajectorySequenceAsync(leave);
                        currentState = state.lineUp;

                    }

                case forwardStack:
                    if(!drive.isBusy()) {
                        double frontSum = 0;
                        int frontCount = 0;

                        for (int x = 0; x < 5; x++) {
                            double distance = bot.distanceSensor.getBotsFrontDistance();
                            if (distance < 58) {
                                frontSum += distance;
                                frontCount++;

                            }
                        }
                        if (frontCount != 0) {
                            frontSum /= frontCount;
                            drive.setPoseEstimate(new Pose2d(-72 + frontSum + 8, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));

                            toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                  //  .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                                  //  .forward(frontSum - 3)
                                 //   .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), -35, Math.toRadians(180)))
                                    // .forward(1)
                                    .waitSeconds(.3)
                                    .back(3)
                                    .addTemporalMarker(1, () -> {
                                        bot.setArmPosition(armState.init, armExtensionState.extending);
                                    })
                                    .addTemporalMarker(() -> {
                                        bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                    })
                                    //.lineToConstantHeading(new Vector2d(-35, -54))
                                    .back(10)

                                    .back(.01)
                                    .resetVelConstraint()
                                    .build();
                        } else {
                            toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                  //  .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))

                                    //  .forward(frontSum - 3)
                                    //   .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), -35, Math.toRadians(180)))
                                    // .forward(1)
                                    .waitSeconds(.3)
                                    .back(3)
                                    .addTemporalMarker(1, () -> {
                                        bot.setArmPosition(armState.init, armExtensionState.extending);
                                    })
                                    .addTemporalMarker(() -> {
                                        bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                                    })
                                    //.lineToConstantHeading(new Vector2d(-35, -54))
                                    .back(10)

                                    .resetVelConstraint()
                                    .build();
                        }
                        drive.followTrajectorySequenceAsync(toStack);
                        //  underTrussOffset = 3;
                        stackDistanceOffset = 0;
                        currentState = state.lineUp;
                    }

                    break;
                case boardFromStack:
                    if(!drive.isBusy()) {
                        TrajectorySequence score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 45, DriveConstants.TRACK_WIDTH))
                                .addTemporalMarker(() -> {
                                    bot.outtakeSlide.setPosition(700);
                                })
                                .lineToLinearHeading(new Pose2d(52.2, -41, Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    bot.lid.setLidPosition(lidState.open);
                                    bot.outtakeSlide.setPosition(825);
                                })
                                .waitSeconds(.4)
                                .resetVelConstraint()
                                .build();
                        drive.followTrajectorySequenceAsync(score);
                        currentState = state.park;
                    }

                    break;

                case park:
                    if(!drive.isBusy()){
                        TrajectorySequence parkInCorner = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(5)
                                .addDisplacementMarker(5,  () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(6,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

                                })

                                .lineToLinearHeading(new Pose2d(46, -45, Math.toRadians(90)))

//

                                .lineToConstantHeading(new Vector2d(52, -57))
                                .build();
                        TrajectorySequence parkNextToBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(5)
                                .addDisplacementMarker(5,  () -> {
                                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                                    bot.setWristPosition(wristState.intaking);
                                })
                                .addDisplacementMarker(10,() -> {
                                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

                                })

                                .lineToLinearHeading(new Pose2d(46, -12, Math.toRadians(90)))


                                .lineToConstantHeading(new Vector2d(52, -10.5))
                                .build();
                        drive.followTrajectorySequenceAsync(parkInCorner);
                        currentState = state.idle;
                    }
                case idle:
                    telemetry.addLine("Inside Idle State");
                    //        telemetry.addData("Tag ID", tagOfInterest.id);
                    telemetry.addData("pose est ", drive.getPoseEstimate());
                    //     telemetry.addData("double y + ", detectYPos);
                    //      telemetry.addData("double y -", detectYNeg);
                    //      telemetry.addData("tag ", tagOfInterest.metadata.fieldPosition.get(1)+6);
                    telemetry.addData("Left Center Dis", bot.distanceSensor.getBotsLeftCenterDistance());
                    telemetry.addData("Left Edge Dis", bot.distanceSensor.getBotsLeftEdgeDistance());
                    telemetry.addData("pose est Y", drive.getPoseEstimate().getY());


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
        aprilTag.setDecimation(4);


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

