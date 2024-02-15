package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths.AprilTags;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.currentState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

@Autonomous(name = "closeRedAprilTag ", group = "goobTest")
public class closeRedAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    String webcamName;
    Robot bot;
    HSVRedDetection redDetection;
    ElapsedTime timer = new ElapsedTime();
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    currentState currentStates;
    Pose2d startPose = new Pose2d(12, 61, Math.toRadians(90));
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6;
    int ID_TAG_OF_INTEREST;
    boolean tagFound = false;

    double leftTapeX = 12, leftTapeY = -33, centerTapeX = 17, centerTapeY = -38, rightTapeX = 22.5, rightTapeY = -39;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        currentStates = currentState.tape;

        telemetry.addLine("April Tag Initialized.");
        telemetry.update();

        // ---------------------------- Tape ---------------------------- //

        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(leftTapeX + 5, leftTapeY - 21))
                .lineToConstantHeading(new Vector2d(leftTapeX + 5, leftTapeY))
                .lineToConstantHeading(new Vector2d(leftTapeX + 3, leftTapeY))
                .lineToLinearHeading(new Pose2d(leftTapeX + 18, leftTapeY + 4.5, Math.toRadians(180)))
                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(centerTapeX, centerTapeY))
                .lineToConstantHeading(new Vector2d(centerTapeX, centerTapeY + 6))
                .lineToLinearHeading(new Pose2d(centerTapeX + 23, centerTapeY + 4, Math.toRadians(180)))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(rightTapeX, rightTapeY))
                .lineToConstantHeading(new Vector2d(rightTapeX - 0.25, rightTapeY - 11))
                .lineToLinearHeading(new Pose2d(rightTapeX + 20.5, rightTapeY - 5, Math.toRadians(180)))
                .build();

        // ---------------------------- Runner ---------------------------- //


        TrajectorySequence tag = null;
        waitForStart();
        initCam();

        switch (redDetection.getLocation()) {
            case LEFT:
                drive.followTrajectorySequenceAsync(leftTape);
                ID_TAG_OF_INTEREST = LEFT;
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(centerTape);
                ID_TAG_OF_INTEREST = MIDDLE;
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(rightTape);
                ID_TAG_OF_INTEREST = RIGHT;
                break;
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            closeCamera();

            switch (currentStates) {
                case tape:
                    telemetry.addLine("Inside Tape State");
                    telemetry.update();

                    initAprilTag();
                    telemetry.addLine("April Tag init");
                    telemetry.update();

                    if (!drive.isBusy()) {
                        currentStates = currentState.firstTimeBoard;
                        timer.reset();
                    }
                    break;

                case firstTimeBoard:
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    if (currentDetections.size() != 0) {
                        for (AprilTagDetection detection : currentDetections) {

                            if (detection.metadata != null) {

                                telemetry.addLine("Inside Metadata If");
                                telemetry.update();

                                //  Check to see if we want to track towards this tag.
                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST)) {
                                    telemetry.addLine("Inside Tag Of Interest If");
                                    telemetry.update();
                                    tagFound = true;
                                    tagOfInterest = detection;
                                }

                                if (tagFound) {
                                    telemetry.addLine("Inside TagFound If Statement");
                                    telemetry.update();

                                    tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.y - 4.5, drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x - 1.65, Math.toRadians(180)))
                                            .build();

                                    telemetry.addData("FTC Pose x: ", tagOfInterest.ftcPose.x);
                                    telemetry.addData("FTC Pose y: ", tagOfInterest.ftcPose.y);
                                    telemetry.addData("New Pose x: ", drive.getPoseEstimate().getX() + tagOfInterest.ftcPose.x);
                                    telemetry.addData("New Pose y: ", drive.getPoseEstimate().getY() + tagOfInterest.ftcPose.y);

                                    telemetry.addLine("Trajectory ran");
                                    telemetry.update();

                                    currentStates = currentState.park;

                                    break;
                                }
                            }
                        }
                    }

                    if (!drive.isBusy())
                    {
                        drive.followTrajectorySequenceAsync(tag);
                        if (tagFound)
                        {
                            currentStates = currentState.idle;
                        }
                        else if (timer.seconds() > 2.5)
                        {
                            currentStates = currentState.idle;
                        }
                    }
                    break;

                case idle:
                    telemetry.addLine("Inside Idle State");
                    telemetry.update();
                    break;

                case park:
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);

                    TrajectorySequence park = drive.trajectorySequenceBuilder(tag.end())
                            .lineToLinearHeading(new Pose2d(50, -53, Math.toRadians(90)))
                            .lineToConstantHeading(new Vector2d(52, -54.5))
                            .build();

                    drive.followTrajectorySequence(park);
                    break;
            }
            drive.update();
        }
    }

    private void initCam() {
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

//                camera.showFpsMeterOnViewport(true);
//
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                //camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
        camera.closeCameraDevice();
    }

    public void closeCamera() {
        if (camera != null) {
            telemetry.addLine("In close camera");

            camera.pauseViewport();

            camera.closeCameraDevice();

        } else {
            telemetry.addLine("Camera is alr null.");
            telemetry.update();
        }

        telemetry.addLine("Pausing/Stopping");
        camera.stopRecordingPipeline();
        camera.pauseViewport();
        camera.closeCameraDevice();
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        telemetry.addLine("Inside April Tag Init");
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

    }
}