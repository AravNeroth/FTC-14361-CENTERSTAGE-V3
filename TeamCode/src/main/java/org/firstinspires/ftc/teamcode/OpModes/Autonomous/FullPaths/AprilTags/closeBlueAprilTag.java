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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

@Autonomous(name = "closeBlueAprilTag ", group = "goobTest")
public class closeBlueAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    NewVision newVision;
    String webcamName;
    Robot bot;
    ElapsedTime timer = new ElapsedTime();
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    currentState currentStates;
    Pose2d startPose = new Pose2d(12, 61, Math.toRadians(90));
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    boolean onePixel = false, twoPixels = false;
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3;
    int ID_TAG_OF_INTEREST = 1;
    boolean tagFound = false;

    double leftTapeX = 22.25, leftTapeY = 55, centerTapeX = 11.5, centerTapeY = 34.5, rightTapeX = 8.5, rightTapeY = 32;
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
                .lineToConstantHeading(new Vector2d(leftTapeX, leftTapeY))
                .lineToConstantHeading(new Vector2d(leftTapeX, leftTapeY - 13))
                .lineToConstantHeading(new Vector2d(leftTapeX, leftTapeY - 5))
                .lineToLinearHeading(new Pose2d(leftTapeX + 20.75, leftTapeY - 12.5, Math.toRadians(180)))
                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(centerTapeX, centerTapeY))
                .lineToConstantHeading(new Vector2d(centerTapeX + 28.5, centerTapeY + 3.5))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(rightTapeX + 6.5, rightTapeY + 22))
                .lineToLinearHeading(new Pose2d(rightTapeX + 6.5, rightTapeY, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(rightTapeX, rightTapeY))
                .lineToConstantHeading(new Vector2d(rightTapeX + 5.5, rightTapeY))
                .lineToLinearHeading(new Pose2d(rightTapeX + 21.5, rightTapeY - 2, Math.toRadians(180)))
                .build();

        // ---------------------------- Runner ---------------------------- //

        TrajectorySequence tag = null;
        waitForStart();

        telemetry.addLine("New Vision Initialized.");
        newColorDetect();

        switch (newVision.getStartingPosition()) {
            case LEFT:
                telemetry.addLine("Left");
                telemetry.update();

                drive.followTrajectorySequenceAsync(leftTape);
                ID_TAG_OF_INTEREST = LEFT;
                break;
            case RIGHT:
                telemetry.addLine("Right");
                telemetry.update();

                drive.followTrajectorySequenceAsync(rightTape);
                ID_TAG_OF_INTEREST = RIGHT;
                break;
            case CENTER:
                telemetry.addLine("Center");
                telemetry.update();

                drive.followTrajectorySequenceAsync(centerTape);
                ID_TAG_OF_INTEREST = MIDDLE;
                break;
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentStates) {
                case tape:
                    telemetry.addLine("Inside Tape State");
                    telemetry.update();

                    if(!aprilTagOn){
                        telemetry.addLine("April Tag Enabled");
                        telemetry.update();
                        initAprilTag();
                        aprilTagOn = true;
                    }

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

                                if ((ID_TAG_OF_INTEREST < 0 || detection.id == ID_TAG_OF_INTEREST)) {
                                    drive.breakFollowing();
                                    telemetry.addLine("Inside Tag Of Interest If");

                                    tagFound = true;
                                    tagOfInterest = detection;
                                }

                                if (tagFound) {
                                    telemetry.addLine("Inside TagFound If Statement");

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
                        if (tagFound)
                        {
                            drive.followTrajectorySequenceAsync(tag);
                            currentStates = currentState.idle;
                            tagFound = false;
                        }
                        else if (timer.seconds() > 2.5)
                        {
                            currentStates = currentState.idle;
                        }
                    }

                    telemetry.update();
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
                            .lineToConstantHeading(new Vector2d(47, 32))
                            .lineToLinearHeading(new Pose2d(47 ,58, Math.toRadians(270)))
                            .build();

                    drive.followTrajectorySequence(park);
                    break;
            }
            drive.update();
        }
    }

    private void newColorDetect(){
        if(opModeIsActive() && !isStopRequested()){
            visionPortal.stopStreaming();
        }
        else
        {
            newVision = new NewVision(telemetry);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(newVision)
                    .enableLiveView(false)
                    .build();

            NewVision.StartingPosition startingPos = NewVision.StartingPosition.LEFT;

            telemetry.addLine("Vision portal built");
            telemetry.addData("Starting position: ", startingPos);
            startingPos = newVision.getStartingPosition();
            telemetry.addData("Called NewVision - returned: ", startingPos);
        }
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