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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Stack.LongRedUltrasonicUnderTruss;
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

@Autonomous(name = "April Tag Heading Test", group = "goobTest")
public class AprilTagHeadingTest extends LinearOpMode {

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
    Pose2d start = new Pose2d(-36.5, -62.75, Math.toRadians(180));
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
    double integralSum = 0;
    double derivative = 0;
    double error = 0;
    double output = 0;
    double lastError = 0;
    ElapsedTime headingTimer = new ElapsedTime();
    boolean timerOn = false;

    state currentState = state.stack;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, stack, idle, park, underGate, lineUp, underTruss, toAprilTags, leaveBoard, setPoseEstimate, underTrussToStack, toStack, boardFromStack, forwardStack, tagHeading
    }

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(37.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
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

                //    .lineToConstantHeading(new Vector2d(-38.25, -40))
                .lineToLinearHeading(new Pose2d(-40, -60, Math.toRadians(181.75)))
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
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                //.turn(Math.toRadians(3))

                .build();
        TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .turn(Math.toRadians(1))
                .build();
        TrajectorySequence turnRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .turn(Math.toRadians(-1))
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
       // newColorDetect();





        telemetry.update();

        TrajectorySequence tag = null;
       initAprilTag();
        waitForStart();


        if (isStopRequested()) return;


        timer.reset();
        ID_TAG_OF_INTEREST = -1;
        currentState = state.toAprilTags;


        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {

                case idle:
                    telemetry.addLine("US Dist" + bot.ultrasonicSensor.getLeftDistanceEdge());
                    telemetry.addLine(" Dist" + bot.distanceSensor.getBotsLeftEdgeDistance());
                    telemetry.update();
                    break;

                case toAprilTags:
                    if(!timerOn){
                        timerOn = true;
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
                                        .turn(Math.toRadians(bot.driveTrain.pidController(180 - detection.ftcPose.elevation)))
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
                                if(!drive.isBusy()){
                                    drive.setPoseEstimate(new Pose2d(tagOfInterest.metadata.fieldPosition.get(0)-8,tagOfInterest.metadata.fieldPosition.get(1), drive.getPoseEstimate().getHeading()));
                                    telemetry.addLine("Reset Pose");
                                    if(timer.seconds() < 2){
                                        drive.followTrajectorySequenceAsync(tag);
                                    } else if (timer.seconds() > 2) {
                                        currentState = state.idle;
                                    }

                                    telemetry.addData("New Pose", drive.getPoseEstimate());
                                  //  currentState = state.toAprilTags;
                                    tagFound = false;
                                }
                                telemetry.update();
                            }
                            else{
                                telemetry.addLine("No tag");
                            }

                        }

                    } // detect for loop end










                    break;
                case forwardStack:
                    if(!drive.isBusy()){
                        if(!timerOn){
                             headingTimer.reset();
                            timerOn = true;
                        }

                        if(timer.seconds() > .1) {
                            timer.reset();
                            double d1 = bot.distanceSensor.getBotsLeftEdgeDistance();
                            double d2 = bot.ultrasonicSensor.getLeftDistanceEdge() ;

                            if (d1 < 50) {
                                double angle = Math.atan(12.25/(Math.abs(d2 - d1)));
                                telemetry.addLine("angle" + Math.toDegrees(angle));
                                telemetry.update();

                                TrajectorySequence turnLeftt = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                        .turn((Math.toRadians(Math.toDegrees(angle) - 90)))
                                        .build();
                                TrajectorySequence turnRightt = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                        .turn((Math.toRadians(90-Math.toDegrees(angle) )))
                                        .build();
                                if(d1 > d2 && d1 - d2 > .5) {
                                    drive.followTrajectorySequenceAsync(turnLeftt);
                                    telemetry.addLine("angle left" + (Math.toDegrees(angle) - 90));
                                }
                                else if(d2 > d1 && d2 - d1 > .5){
                                   drive.followTrajectorySequenceAsync(turnRightt);
                                    telemetry.addLine("angle right" + (90-Math.toDegrees(angle)));
                                }
                                else if(headingTimer.seconds() > 2){
                                    currentState = state.idle;
                                }
                                else{
                                    currentState = state.idle;
                                }

                                timer.reset();
                            }
                        }
                    }
                    break;
                case tagHeading:
                    if(!drive.isBusy()){
                        if(!timerOn){
                            headingTimer.reset();
                            timerOn = true;
                        }

                        if(timer.seconds() > .1) {
                            timer.reset();
                            double d1 = bot.distanceSensor.getBotsLeftEdgeDistance();
                            double d2 = bot.ultrasonicSensor.getLeftDistanceEdge() ;

                            if (d1 < 50) {
                                double angle = Math.atan(12.25/(Math.abs(d2 - d1)));
                                telemetry.addLine("angle" + Math.toDegrees(angle));
                                telemetry.update();

                                TrajectorySequence turn = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                        .turn((Math.toRadians(90-Math.toDegrees(angle) )))
                                        .build();
                                 if(Math.abs(d1-d2) > .5){
                                    drive.followTrajectorySequenceAsync(turn);
                                 }
                                else if(headingTimer.seconds() > 2){
                                    currentState = state.idle;
                                }
                                else{
                                     //   currentState = state.idle;
                                }

                                timer.reset();
                            }
                        }
                    }
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


