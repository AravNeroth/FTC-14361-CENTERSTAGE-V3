package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "moreStateTesting ", group = "goobTest")
public class moreStateTesting extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    String webcamName;
    Robot robot;
    ElapsedTime intake = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, STACK = 7;
    int ID_TAG_OF_INTEREST = -1;
    boolean tagFound = false;
    boolean caseTagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 0, centerTapeY = 0, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    HSVBlueDetection blueDetection;


    state currentState = state.tape;


    enum state {
        tape, firstTimeBoard, secondTimeBoard, thirdTimeBoard, stack
    }

    AprilTagDetection lastTOI = null;
    double initialDistance, y;


    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        initAprilTag();

        telemetry.addLine("April Tag Initialized.");
        telemetry.update();

//
//        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(leftTapeX,leftTapeY))
//                .build();
//
//        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
//                .lineToConstantHeading(new Vector2d(centerTapeX,centerTapeY))
//                .build();
//
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
        waitForStart();


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            //telemetryAprilTag();
            //telemetry.update();

            switch (currentState) {

                case tape:
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();



                    if (currentDetections.size() != 0) {


                        for (AprilTagDetection detection : currentDetections) {

                                if (detection.metadata != null) {

                                    telemetry.addLine("Inside Metadata If");
                                    telemetry.update();

                                    //  Check to see if we want to track towards this tag.
                                    if ((ID_TAG_OF_INTEREST < 0)) {
                                        telemetry.addLine("Inside Tag Of Interest If");
                                        telemetry.update();
                                        // Yes, we want to use this tag.
                                        tagFound = true;
                                        tagOfInterest = detection;
                                    }

                                    if (tagFound) {
                                        telemetry.addLine("Inside TagFound If Statement");
                                        telemetry.update();

                                        TrajectorySequence tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .lineToConstantHeading(new Vector2d(tagOfInterest.ftcPose.x - .5, tagOfInterest.ftcPose.y))
                                                .build();

                                        drive.followTrajectorySequence(tag);

                                        telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.x);
                                        telemetry.addData("FTC Pose y: ", tagOfInterest.ftcPose.y);

                                        telemetry.addLine("Traj Seq Builder ran");
                                        telemetry.update();
                                    }
                                }

                                else {
                                    telemetry.addLine("Not found");
                                    telemetry.update();

                                }

                                ID_TAG_OF_INTEREST = STACK;
                                currentState = state.stack;

                        } // detect for loop end

                    } // if detect not 0 end
                    break;
                case stack:

            } //switch statement end


            } // opmode loop

        } // run opmode
        private void initCam(){

            //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcamName = "Webcam 1";

            // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
            // The camera instance is stored in the camera variable that we can use later
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

            // initializing our Detection class (details on how it works at the top)
            blueDetection = new HSVBlueDetection(telemetry);

            // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
            camera.setPipeline(blueDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.showFpsMeterOnViewport(true);
                    camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addLine("Unspecified Error Occurred; Camera Opening");
                }
            });
        }


        /**
         * Add telemetry about AprilTag detections.
         */

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
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


    }
// end class







