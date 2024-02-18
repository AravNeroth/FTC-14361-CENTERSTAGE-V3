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

@Autonomous(name = "Stack Tag Testing", group = "goobTest")
public class StackAprilTagTesting extends LinearOpMode {

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
    Pose2d start = new Pose2d(-39, -40.5, Math.toRadians(0));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, underTrussBool = false, stackBool = false, finishBoard = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    double tagX = 60.275, centerTagY = -29.4, leftTagY = -24.4, rightTagY = -34.4;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 8;
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




        TrajectorySequence goToStackTag = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(18, 45, DriveConstants.TRACK_WIDTH))
                .strafeLeft(15)

                .resetVelConstraint()
                        .build();

                //   .strafeRight(3)



        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        telemetry.addLine("portal state " + visionPortal.getCameraState());


        telemetry.update();

        TrajectorySequence tag = null;

        waitForStart();

initAprilTag();
        if (isStopRequested()) return;
        currentState = state.tape;
        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case tape:
                    if(!drive.isBusy()){
                        initStackAprilTag();
                        currentState = state.firstTimeBoard;
                    }
                case firstTimeBoard:
                    telemetry.addLine("drive" + drive.isBusy());
                    telemetry.update();

                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                            if (detection.metadata != null) {

                              //  telemetry.addLine("Inside Metadata If");
                              //  telemetry.update();

                                //  Check to see if we want to track towards this tag.
                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST) && !finishBoard) {
                                    drive.breakFollowing();
                              //      telemetry.addLine("Inside Tag Of Interest If");
                             //       telemetry.update();
                                    // Yes, we want to use this tag.
                                   tagY = drive.getPoseEstimate().getY() + detection.ftcPose.y;
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
                          //      telemetry.addLine("Inside TagFound If Statement");
                            //    telemetry.update();
                                timer.reset();
                                temporalMarkerTimer.reset();
                                // final double distanceX = tagOfInterest.center.x;

                                //   tagY = drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.y-3;
                                tag = drive.trajectorySequenceBuilder(start)
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(7.5, 45, DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.x + 4, tagY-.5))
                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - tagOfInterest.ftcPose.x + 4, tagY + 5))
                                        .waitSeconds(.15)
                                        //   .lineToConstantHeading(new Vector2d(50, tagY - 8))

                                        .build();

                                //field pos gives a pos number for red
                                //ftc.pose.y is always positive
                                //On the negative side of x, the tag x vallue is negative
                                telemetry.addData("FTC Pose X: ", tagOfInterest.ftcPose.x);
                                telemetry.addData("FTC Pose Y: ", tagOfInterest.ftcPose.y);
                                telemetry.addData("Field Pose ", tagOfInterest.metadata.fieldPosition);
                                telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);
                                telemetry.addData("Tag y: -", drive.getPoseEstimate().getY() + (-tagOfInterest.ftcPose.y));
                                telemetry.addData("Tag y: +", drive.getPoseEstimate().getY() - (-tagOfInterest.ftcPose.y));
                                telemetry.addData(" Field Pos -" ,-tagOfInterest.metadata.fieldPosition.get(1));
                                telemetry.addData("- pose x " ,-tagOfInterest.ftcPose.x);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);

                                telemetry.addLine("Traj Seq Builder ran");
                                telemetry.update();
                            } else
                            {
                                telemetry.addData("Different Tag Found", detection.id);
                                telemetry.addData("Different Tag X", detection.ftcPose.x);
                                telemetry.addData("Different Tag Y", detection.ftcPose.y);
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
                       // telemetry.update();
                        if(!finishBoard){
                        //    drive.followTrajectorySequenceAsync(tag);
                            finishBoard = true;
                        }
                        if(!drive.isBusy()){
                         //   drive.setPoseEstimate(new Pose2d(tagOfInterest.metadata.fieldPosition.get(0)+10,tagOfInterest.metadata.fieldPosition.get(1)+ 5.5, Math.toRadians(180)));
                            telemetry.addLine("Reset Pose");
                            telemetry.addData("NewPose", drive.getPoseEstimate());
                          // currentState = state.idle;
                        }

                    }
//                    else if (timer.seconds() > 9) {
//                        currentState = state.idle;
//                    }
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
    private void initStackAprilTag() {
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

