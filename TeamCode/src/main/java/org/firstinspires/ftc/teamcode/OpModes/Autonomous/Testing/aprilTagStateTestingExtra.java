package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable.NewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable.TestNewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.HSVRedDetection;
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

@Autonomous(name = "aprilTagStateTesting ", group = "goobTest")
public class aprilTagStateTestingExtra extends LinearOpMode {

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
    TestNewVision testNewVision;


    String webcamName;
    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(11.5, -62.75, Math.toRadians(270));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 4;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;




    state currentState = state.tape;


    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack, idle
    }






    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(centerTapeX, centerTapeY))
                .lineToConstantHeading(new Vector2d(centerTapeX, centerTapeY - 5))
                .turn(Math.toRadians(-90))

                .build();
        TrajectorySequence goTowardsAprilTags = drive.trajectorySequenceBuilder(centerTape.end())
                .lineToConstantHeading(new Vector2d(centerTapeX + 3, centerTapeY - 5))
                .strafeRight(3)
                .build();


        telemetry.addLine("April Tag Initialized.");
     //   newColorDetect();
        newNewColorVision();
        telemetry.addLine("portal state " + visionPortal.getCameraState());
     //   telemetry.addLine("portal active " + visionPortal.getActiveCamera());
//        switch (newVision.getLocation()){
//            case LEFT:
//                drive.followTrajectorySequenceAsync(centerTape);
//                ID_TAG_OF_INTEREST = LEFT;
//                break;
//            case RIGHT:
//                drive.followTrajectorySequenceAsync(centerTape);
//                ID_TAG_OF_INTEREST = LEFT;
//                break;
//            case MIDDLE:
//                drive.followTrajectorySequenceAsync(centerTape);
//                ID_TAG_OF_INTEREST = LEFT;
//                break;
//        }

        telemetry.update();

//
//        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(leftTapeX,leftTapeY))
//                .build();


//        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(rightTapeX,rightTapeY))
//                .build();
//        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(leftBoardX,leftBoardY))
//                .build();
//        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(centerBoardX,centerBoardY))
//                .build();
//        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(rightBoardX,rightBoardY))
//                .build();



        // initCam();
        TrajectorySequence tag = null;
        waitForStart();


        if (isStopRequested()) return;
       // drive.followTrajectorySequenceAsync(forward);
       // currentState = state.tape;
        while (opModeIsActive() && !isStopRequested()) {
         //   closeCamera();
            //camera.stopStreaming();
         //   camera.closeCameraDevice();


            //telemetryAprilTag();
            //telemetry.update();

            switch (currentState) {
                case tape:
                if(!cameraOn){
                    //newColorDetect();
                    telemetry.addLine("into disalbe");
                    telemetry.update();
                    cameraOn = true;
                    timer.reset();
                }
                if(!aprilTagOn){
                    telemetry.addLine("into april tag disable");
                    telemetry.update();
                    initAprilTag();
                    aprilTagOn = true;
                }



//                    if (!drive.isBusy()) {
//                        currentState = state.firstTimeBoard;
//                        timer.reset();
//                    }
//
//                        // drive.followTrajectoryAsync(trajectory2);
//                    }
                    break;
                case firstTimeBoard:
                    if(!toAprilTag1){
                        drive.followTrajectorySequenceAsync(goTowardsAprilTags);
                        toAprilTag1 = false;

                    }


                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                            if (detection.metadata != null) {

                                telemetry.addLine("Inside Metadata If");
                                telemetry.update();

                                //  Check to see if we want to track towards this tag.
                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST)) {
                                    drive.breakFollowing();
                                    telemetry.addLine("Inside Tag Of Interest If");
                                    telemetry.update();
                                    // Yes, we want to use this tag.

                                    tagFound = true;
                                    tagOfInterest = detection;
                                }

                                if (tagFound) {
                                    telemetry.addLine("Inside TagFound If Statement");
                                    telemetry.update();
                                    timer.reset();
                                    // final double distanceX = tagOfInterest.center.x;
                                    tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX()  + tagOfInterest.ftcPose.y +3, drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.x+4))
                                            .build();



                                    telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.x);
                                    telemetry.addData("FTC Pose y: ", tagOfInterest.ftcPose.y);
                                    telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x);
                                    telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);

                                    telemetry.addLine("Traj Seq Builder ran");
                                    telemetry.update();
                                }
                            }



                            //    ID_TAG_OF_INTEREST = REDSTACK;


                        } // detect for loop end

                    } // if detect not 0 end
//                    else{
//                        tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .lineToConstantHeading(new Vector2d(1, 1))
//                                .build();
//                    }
                    if (!drive.isBusy()) {

                        if (tagFound) {
                            drive.followTrajectorySequenceAsync(tag);
                            currentState = state.idle;
                            tagFound =false;

                        } else if (timer.seconds() > 2.5) {
                            currentState = state.idle;
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



    private void newColorDetect(){
        if(opModeIsActive()){
            visionPortal.stopStreaming();
        }
        else {
            newVision = new NewVision(telemetry);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(newVision)
                    .enableLiveView(false)
                    // .addProcessor(newVision)
                    .build();

//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), newVision);
        }
    }
    private void newNewColorVision(){
        if(opModeIsActive()){
            visionPortal.stopStreaming();
        }
        else {
            testNewVision = new TestNewVision(telemetry);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(testNewVision)
                    .enableLiveView(false)
                    // .addProcessor(newVision)
                    .build();

//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), newVision);
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


        //   camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);

    }





}
// end class
