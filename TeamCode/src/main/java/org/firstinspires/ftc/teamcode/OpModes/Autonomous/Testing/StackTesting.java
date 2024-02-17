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

@Autonomous(name = "Stack Testing ", group = "goobTest")
public class StackTesting extends LinearOpMode {

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
    Pose2d start = new Pose2d(-36.5, -62.75, Math.toRadians(180));
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
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    state currentState = state.tape;

    enum state {
        tape, underTruss,firstTimeBoard, secondTimeBoard, thirdTimeBoard, toStack, idle,leaveStack
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToConstantHeading(new Vector2d(-38.25, -37))
                .addTemporalMarker(.05,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .addTemporalMarker(.15,() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(-38.25, -40))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(38, 45, DriveConstants.TRACK_WIDTH))

                .lineToLinearHeading(new Pose2d(-38.25 ,-59.75, Math.toRadians(180)))
                .waitSeconds(.25)

                .resetVelConstraint()
                .build();
        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(centerTape.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(48, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(30, -58))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .waitSeconds(.25)
                .resetVelConstraint()
                //   .strafeRight(3)
                .build();
        TrajectorySequence underTrussToStack = drive.trajectorySequenceBuilder(new Pose2d(-40, -51.5, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 45, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-57,-37.7, Math.toRadians(230)))
                .addTemporalMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.81);

                })
              //   .forward(.25)
                .turn(Math.toRadians(-100))
                .turn(Math.toRadians(50))
                .addTemporalMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                   bot.setActiveIntakePosition(activeIntakeState.active);

                })
                // .strafeRight(5)
                .forward(.35)
                .waitSeconds(1)



//                .lineToConstantHeading(new Vector2d(50, -61))

                .resetVelConstraint()
                //   .strafeRight(3)
                .build();
        TrajectorySequence leaveStack = drive.trajectorySequenceBuilder((underTrussToStack.end()))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-55,-28.5, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })

                .back(1)
                .addTemporalMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
//                .lineToConstantHeading(new Vector2d(-40, -54))
//                .addTemporalMarker(() -> {
//                    bot.setActiveIntakePosition(activeIntakeState.inactive);
//                })
//                .lineToConstantHeading(new Vector2d(40, -53))




//                .lineToConstantHeading(new Vector2d(50, -61))

                .resetVelConstraint()
                //   .strafeRight(3)
                .build();


        TrajectorySequence goToCenterAprilTag = drive.trajectorySequenceBuilder(underTruss.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(18, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(32.5, -25))

                .resetVelConstraint()

                //   .strafeRight(3)
                .build();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());

        switch (newVision.getStartingPosition()){
            case LEFT:
                temporalMarkerTimer.reset();
           //     drive.followTrajectorySequenceAsync(centerTape);
                telemetry.addLine("left.");
                telemetry.update();
                ID_TAG_OF_INTEREST = LEFT;
                break;
            case RIGHT:
                temporalMarkerTimer.reset();
             //   drive.followTrajectorySequenceAsync(centerTape);
                telemetry.addLine("right.");
                telemetry.update();
                ID_TAG_OF_INTEREST = RIGHT;
                break;
            case CENTER:
                temporalMarkerTimer.reset();
              //  drive.followTrajectorySequenceAsync(centerTape);
                telemetry.addLine("center.");
                telemetry.update();
                ID_TAG_OF_INTEREST = MIDDLE;
                break;
        }

        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
        currentState = state.toStack;
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
                case underTruss:
                    if(!underTrussBool){
                        drive.followTrajectorySequenceAsync(underTruss);
                        underTrussBool = true;
                    }

                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(goToCenterAprilTag);
                        timer.reset();
                        currentState = state.firstTimeBoard;
                    }
                    break;
                case firstTimeBoard:
                    telemetry.addLine("drive" + drive.isBusy());
                    telemetry.update();

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
                                    if(drive.getPoseEstimate().getY() - detection.metadata.fieldPosition.get(1) > 0){
                                        tagY = drive.getPoseEstimate().getY() - (-detection.ftcPose.x);
                                    }
                                    else {
                                        tagY = drive.getPoseEstimate().getY() + (-detection.ftcPose.x);
                                    }
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
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 45, DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y -2, tagY - 8.6))
                                        .addDisplacementMarker( 1, () -> {
                                            bot.outtakeSlide.setPosition(650);
                                        })
                                        .addTemporalMarker( () -> {
                                            bot.setLidPosition(lidState.open);
                                            bot.outtakeSlide.setPosition(775);
                                        })
                                        .waitSeconds(.1)
                                        //   .lineToConstantHeading(new Vector2d(50, tagY - 8))

                                        .build();


                                telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.x);
                                telemetry.addData("FTC Pose y: ", tagOfInterest.ftcPose.y);
                                telemetry.addData("Field Pose ", tagOfInterest.metadata.fieldPosition);
                                telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);

                                telemetry.addLine("Traj Seq Builder ran");
                                telemetry.update();
                            } else
                            {
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
                        telemetry.update();
                        if(!finishBoard){
                            drive.followTrajectorySequenceAsync(tag);
                            finishBoard = true;
                        }
                        if(!drive.isBusy()){
                            drive.setPoseEstimate(new Pose2d(tagOfInterest.metadata.fieldPosition.get(0)-8,tagOfInterest.metadata.fieldPosition.get(1), Math.toRadians(180)));
                            telemetry.addLine("Reset Pose");
                            telemetry.addData("NewPose", drive.getPoseEstimate());
                            currentState = state.toStack;
                        }

                    }
                    else if (timer.seconds() > 9) {
                        currentState = state.idle;
                    }




                    //  currentState = state.firstTimeBoard;


                    break;
                case toStack:
                    telemetry.addData("NewPose", drive.getPoseEstimate());
                    telemetry.update();
                    if(!stackBool){
                        if(!drive.isBusy()){
                            drive.followTrajectorySequenceAsync(underTrussToStack);
                            currentState = state.leaveStack;
                            stackBool = true;
                        }


                    }
//                    if(!drive.isBusy()){
//                        currentState = state.idle;
//                    }
                    break;
                case leaveStack:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(leaveStack);
                        currentState = state.idle;
                    }
                    break;
                case idle:
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
