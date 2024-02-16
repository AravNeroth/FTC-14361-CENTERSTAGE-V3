package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
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

import java.util.List;
@Autonomous(name = "testingAprilTagFunctions ", group = "goobTest")
public class TestingAprilTagFunctions extends LinearOpMode{
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
    double tagY = 0;


    String webcamName;
    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(25, -31, Math.toRadians(270));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    TrajectorySequence turnToAprilTag = null;

    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 5;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    aprilTagStateTestingExtra.state currentState = aprilTagStateTestingExtra.state.tape;

    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack, idle
    }

    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                //.lineToConstantHeading(new Vector2d(19,-55))
                .lineToConstantHeading(new Vector2d(11.5, -39))
                .lineToLinearHeading(new Pose2d(30, -34, Math.toRadians(180)))
                .build();

        TrajectorySequence goTowardsAprilTags = drive.trajectorySequenceBuilder(centerTape.end())
                .lineToConstantHeading(new Vector2d(centerTapeX + 3, centerTapeY - 7))
                .strafeRight(3)
                .build();
        TrajectorySequence turn = drive.trajectorySequenceBuilder(start)
                .turn(Math.toRadians(-90))
                .build();




      //  telemetry.addLine("portal state " + visionPortal.getCameraState());
        initAprilTag();

        telemetry.update();

        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
        currentState = aprilTagStateTestingExtra.state.tape;
        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case tape:
                    if(!toAprilTag1){
                        drive.followTrajectorySequenceAsync(turn);
                        toAprilTag1 = true;
                    }

List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                            if (detection.metadata != null) {

                                telemetry.addLine("Inside Metadata If");
                                telemetry.update();

                                //  Check to see if we want to track towards this tag.
                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST)) {
                                 //   drive.breakFollowing();
                                    telemetry.addLine("Inside Tag Of Interest If");
                                    telemetry.update();
                                    // Yes, we want to use this tag.

                                    tagFound = true;
                                    tagOfInterest = detection;
                                    if(drive.getPoseEstimate().getY() - detection.metadata.fieldPosition.get(1) > 0){
                                        tagY = drive.getPoseEstimate().getY() - tagOfInterest.ftcPose.x;
                                    }
                                    else {
                                        tagY = drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.x;
                                    }

                                }
                                else if (!randomTag) {
//                                    TrajectorySequence turnToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                            .turn(detection.ftcPose.bearing)
//                                            .build();

                                    randomTag = true;
                                }
                            }

                            if (tagFound) {
                                telemetry.addLine("Inside TagFound If Statement");
                                telemetry.update();
                                timer.reset();
                                double currentAngle = tagOfInterest.ftcPose.elevation;
                                 turnToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                         .lineToConstantHeading(new Vector2d(25, -31 - tagOfInterest.ftcPose.x-4))
                                        .turn(-Math.toRadians(currentAngle))
                                         .waitSeconds(1)
                                        .build();

//                                tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.x))
//                                        .turn(Math.toRadians(-tagOfInterest.ftcPose.elevation))
//                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.x))
//                                        .build();


                                telemetry.addData("FTC Pose y: ", tagY);
                                telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.y);
                                telemetry.addData("Elevation: ", tagOfInterest.ftcPose.elevation);
                                telemetry.addData("Field Pose ", tagOfInterest.metadata.fieldPosition);
                                telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.x);

                                telemetry.addLine("Traj Seq Builder ran");
                                telemetry.update();
                            } else {
                                telemetry.addData("Different Tag Found", detection.id);
                                telemetry.addData("Different Tag X", detection.ftcPose.y);
                                telemetry.addData("Different Tag Y", detection.ftcPose.x);
                                telemetry.addData("Bearing", detection.ftcPose.bearing);
                                telemetry.addData("Elevation", detection.ftcPose.elevation);
                                if(detection.metadata!= null){
                                    telemetry.addData("Field Pose ", detection.metadata.fieldPosition);
                                }

                                telemetry.update();
                            }
                        }
                        if (!drive.isBusy()) {

                            if (!aprilTagOn) {
                             //   drive.followTrajectorySequenceAsync();
                                   drive.followTrajectorySequenceAsync(turnToAprilTag);
                                aprilTagOn = true;
                              //  currentState = aprilTagStateTestingExtra.state.idle;
                                tagFound = false;

                            } else if (timer.seconds() > 10) {
                                currentState = aprilTagStateTestingExtra.state.idle;
                            }

                            //  currentState = state.firstTimeBoard;
                        }


                        //    ID_TAG_OF_INTEREST = REDSTACK;


                    } // detect for loop end
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case firstTimeBoard:


                    // if detect not 0 end
//                    else{
//                        tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .lineToConstantHeading(new Vector2d(1, 1))
//                                .build();
//                    }
                    if (!drive.isBusy()) {

                        if (tagFound) {
                            drive.followTrajectorySequenceAsync(tag);
                            currentState = aprilTagStateTestingExtra.state.idle;
                            tagFound = false;

                        } else if (timer.seconds() > 3.5) {
                            currentState = aprilTagStateTestingExtra.state.idle;
                        }

                        //  currentState = state.firstTimeBoard;
                    }

                    break;
                case idle:
                    telemetry.addLine("Inside Idle State");
                    telemetry.update();
                    break;
            } //switch statement end

            drive.update();
        } // opmode loop

    } // run opmode


    private void newColorDetect() {
        if (initCam) {
            visionPortal.stopStreaming();
        } else {
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
