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

@Autonomous(name = "Distance Sensor Testing", group = "goobTest")
public class DistanceSensorTesting extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    double distanceFromEdge = 3.5, distanceFromCenter = 4;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    NewVision newVision;


    String webcamName;
    Robot bot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime temporalMarkerTimer = new ElapsedTime();
    Pose2d start = new Pose2d(-36.5, -39.5, Math.toRadians(180));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, underTrussBool = false, stackBool = false, finishBoard = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    double centerTagX = 60.275, centerTagY = -29.4;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 4;
    double batteryOffset = 0;
    boolean tagFound = false;
    double offset = 0;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    state currentState = state.tape;

    enum state {
        tape, underTruss,firstTimeBoard, secondTimeBoard, thirdTimeBoard, toStack, idle,leaveStack, park
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToConstantHeading(new Vector2d(-39.25, -36.4))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-39.25, -40))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))

                .lineToLinearHeading(new Pose2d(-39.25 ,-60, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                //.lineToConstantHeading(new Vector2d(19,-55))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-42, -38))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-42, -43))



                .lineToLinearHeading(new Pose2d(-40 ,-60, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))
                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToLinearHeading(new Pose2d(-39.25,-42, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-34, -42))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-39.25, -42))
                // .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-39.25, -57))
                // .lineToLinearHeading(new Pose2d(-38.25 ,-60, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(centerTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(30, -55.5))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .waitSeconds(.05)
                .resetVelConstraint()
                //   .strafeRight(3)
                .build();
        TrajectorySequence underTrussToStack = drive.trajectorySequenceBuilder(new Pose2d(centerTagX-8, centerTagY, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(40, centerTagY))
                .addTemporalMarker(.25,() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })
                .addTemporalMarker(.4,() -> {

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
                .addDisplacementMarker(1,() -> {
                    bot.setLinkagePosition(linkageState.LOW);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                //  .waitSeconds(.25)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-55,-28.5, Math.toRadians(180)))
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





        TrajectorySequence goToCenterAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(32.5, -41))

                .resetVelConstraint()

                //   .strafeRight(3)
                .build();
        TrajectorySequence goToLeftAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(32.5, -46))

                .resetVelConstraint()

                //   .strafeRight(3)
                .build();
        TrajectorySequence goToRightAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(32.5, -36))

                .resetVelConstraint()

                //   .strafeRight(3)
                .build();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());



        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
//        switch (newVision.getStartingPosition()){
//            case CENTER:
//                ID_TAG_OF_INTEREST = MIDDLE;
//                temporalMarkerTimer.reset();
//                drive.followTrajectorySequenceAsync(centerTape);
//                telemetry.addLine("CENTER.");
//                telemetry.update();
//
//                break;
//            case RIGHT:
//                ID_TAG_OF_INTEREST = RIGHT;
//                temporalMarkerTimer.reset();
//
//                drive.followTrajectorySequenceAsync(rightTape);
//                telemetry.addLine("right.");
//                telemetry.update();
//
//                break;
//            case LEFT:
//                ID_TAG_OF_INTEREST = LEFT;
//                temporalMarkerTimer.reset();
//
//                drive.followTrajectorySequenceAsync(leftTape);
//                telemetry.addLine("left.");
//                telemetry.update();
//
//                break;
//        }
        currentState = state.park;
        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case tape:
                    if(!cameraOn){

                        // newColorDetect();
                        telemetry.addLine("Into disalbe");
                        telemetry.update();
                        cameraOn = true;
                        timer.reset();


                    }
                    if(!aprilTagOn){
                        telemetry.addLine("into april tag enable");
                        telemetry.update();
                        initAprilTag();
                        aprilTagOn = true;
                    }



                    if (!drive.isBusy()) {
                        // drive.followTrajectorySequenceAsync(goToCenterAprilTag);
                        currentState = state.underTruss;
                        temporalMarkerTimer.reset();
                        timer.reset();
                    }
//
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case park:
                    if(!drive.isBusy()){
                        TrajectorySequence lineUpWithWall = drive.trajectorySequenceBuilder(start)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 45, DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - bot.distanceSensor.getLeftDistanceEdgeDistance() + 3.75))


                                //   .strafeRight(3)
                                .build();
                        drive.followTrajectorySequence(lineUpWithWall);
                        currentState = state.idle;
                    }
                        break;
                case underTruss:
                    if(!underTrussBool){
                        drive.followTrajectorySequenceAsync(underTruss);
                        underTrussBool = true;
                    }

                    if(!drive.isBusy()) {
                        if (!drive.isBusy()) {
                            if (ID_TAG_OF_INTEREST == MIDDLE) {
                                drive.followTrajectorySequenceAsync(goToCenterAprilTag);
                                currentState = state.firstTimeBoard;
                                offset = 6;
                            } else if (ID_TAG_OF_INTEREST == LEFT) {
                                drive.followTrajectorySequenceAsync(goToLeftAprilTag);
                                currentState = state.firstTimeBoard;
                                offset = 3.5;
                            } else if (ID_TAG_OF_INTEREST == RIGHT) {
                                drive.followTrajectorySequenceAsync(goToRightAprilTag);
                                currentState = state.firstTimeBoard;
                                offset = 5;
                            }

                            timer.reset();

                        }
                    }
                    break;

                case idle:


                    telemetry.addLine("Inside Idle State");
                    telemetry.addData("NewPose", drive.getPoseEstimate());
                    telemetry.addData("Distance",bot.distanceSensor.getLeftDistance());
                    telemetry.addData("Bot's Distance",bot.distanceSensor.getLeftDistanceEdgeDistance());
                    telemetry.addData("Bot's Distance",bot.distanceSensor.getBotsLeftCenterDistance());
                    telemetry.update();
                    break;
            } //switch statement end

            drive.update();
            telemetry.addData("NewPose", drive.getPoseEstimate());
            telemetry.addData("Distance",bot.distanceSensor.getLeftDistance());
            telemetry.addData("Bot's Distance",bot.distanceSensor.getLeftDistanceEdgeDistance());
            telemetry.addData("Bot's Distance",bot.distanceSensor.getBotsLeftCenterDistance());
            telemetry.update();
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
        aprilTag.setDecimation(2);


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
