package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.TagDetection;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "More April Tag Testing", group = "goobTest")
public class moreAprilTagTests extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    TagDetection aprilTagDetect;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    String webcamName;
    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(90));
    //Pose2d start = new Pose2d(0, 0, Math.toRadians(90));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 4;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = 34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    HSVRedDetection redDetection;


    state currentState = state.tape;


    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack, idle,
    }


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);


        // telemetry.addLine("April Tag Initialized.");
        //  telemetry.update();

//
//        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(leftTapeX,leftTapeY))
//                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(0, -10))
                .lineToConstantHeading(new Vector2d(0, -10 + 5))
                .turn(Math.toRadians(-90))
                .build();

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
        // telemetry.addLine("camera init ??? hello");


        if (isStopRequested()) return;
        // drive.followTrajectorySequenceAsync(forward);
        initCam();
        waitForStart();
        currentState = state.tape;
        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case tape:

                    telemetry.addLine("Inside Tape State");
                    telemetry.update();
                    if (!drive.isBusy()) {
                        //camera.stopStreaming();
                        initAprilTag();
                        currentState = state.firstTimeBoard;
                        timer.reset();
                    }
                    break;
                case firstTimeBoard:
                    telemetry.addLine("BOARD");
                    telemetry.addLine("timer" + timer.seconds());
                    telemetry.update();

                    ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();

                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                            //  Check to see if we want to track towards this tag.
                            if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST)) {
                                telemetry.addLine("Inside Tag Of Interest If");
                                telemetry.update();
                                // Yes, we want to use this tag.
                                tagFound = true;
                                tagOfInterest = detection;
                            }

                            if (tagFound) {
                                telemetry.addLine("Inside TagFound If Statement");
                                telemetry.update();
                                // final double distanceX = tagOfInterest.center.x;
                                tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + tagOfInterest.pose.y - 4.5, drive.getPoseEstimate().getY() + tagOfInterest.pose.x - 1.65, Math.toRadians(0)))
                                        .build();

                                telemetry.addData("FTC Pose x: ", tagOfInterest.pose.x);
                                telemetry.addData("FTC Pose y: ", tagOfInterest.pose.y);
                                telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + 100* tagOfInterest.pose.x);
                                telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + 100* tagOfInterest.pose.y);

                                telemetry.addLine("Traj Seq Builder ran");
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
                    if (!drive.isBusy()) {

                        if (tagFound) {
                            //      drive.followTrajectorySequenceAsync(tag);
                        }
//                            currentState = state.idle;
//
//                        } else if (timer.seconds() > 2.5) {
//                            currentState = state.idle;
//                        }

                        //  currentState = state.firstTimeBoard;
                    }

                    break;
                case idle:
                    telemetry.addLine("Inside Idle State");
                    telemetry.update();
                    break;
            } //switch statement end

            drive.update();
        }
    } // opmode loop

    // run opmode
    private void initCam (){


        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.SOFTWARE);
        // initializing our Detection class (details on how it works at the top)
        redDetection = new HSVRedDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(redDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.showFpsMeterOnViewport(true);

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addLine("Camera Streaming");
                //          camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");

            }


        });
        //    OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


    } // init cam end



    private void initAprilTag () {
        // Create the AprilTag processor by using a builder.
        aprilTagDetect = new TagDetection(tagsize, 578.272, 578.272, 402.145, 221.506);
        camera.setPipeline(aprilTagDetect);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        //   camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);

    }
    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == ID_TAG_OF_INTEREST) {
                    tagOfInterest = tag;
                    //telemetry.addLine("FOUND!!");
                    tagFound = true;
                    break;
                }
            }
        }
    }

}
// end class