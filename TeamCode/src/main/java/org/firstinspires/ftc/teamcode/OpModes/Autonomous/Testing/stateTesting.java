package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "StateTesting ", group = "goobTest")
public class stateTesting extends LinearOpMode {

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
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    org.openftc.apriltag.AprilTagDetection tagOfInterest = null;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3, STACK = 7;
    int ID_TAG_OF_INTEREST = -1;
    boolean tagFound = false;
    boolean caseTagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 0, centerTapeY = 0, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, centerBoardX, centerBoardY, rightBoardX, rightBoardY;
    double   secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;

    HSVBlueDetection blueDetection;



    state currentState = state.tape;



    enum state {
        tape, firstTimeBoard,secondTimeBoard,thirdTimeBoard,stack
    }

    AprilTagDetection lastTOI = null;
    double initialDistance, y;




    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(leftTapeX,leftTapeY))
                .build();

        TrajectorySequence centerTape = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(centerTapeX,centerTapeY))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(rightTapeX,rightTapeY))
                .build();
        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(leftBoardX,leftBoardY))
                .build();
        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(centerBoardX,centerBoardY))
                .build();
        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(rightBoardX,rightBoardY))
                .build();



        initCam();
        waitForStart();
        camera.stopStreaming();


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case tape:
                    switch (blueDetection.getLocation())
                    {
                        case LEFT:
                            drive.followTrajectorySequenceAsync(leftTape);
                            ID_TAG_OF_INTEREST = LEFT;
                        case RIGHT:
                            drive.followTrajectorySequenceAsync(rightTape);
                            ID_TAG_OF_INTEREST = RIGHT;
                        case MIDDLE:
                            drive.followTrajectorySequenceAsync(centerTape);
                            ID_TAG_OF_INTEREST = MIDDLE;
                    }
                    currentState = state.firstTimeBoard;

                    break;
                case firstTimeBoard:
                    ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                    if(currentDetections.size() != 0)
                    {

                        for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                        {
                            if(tag.id == ID_TAG_OF_INTEREST)
                            {
                                tagOfInterest = tag;
                                tagFound = true;
                                break;
                            }
                        }
                        if(tagFound){
                            TrajectorySequence tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineTo(new Vector2d(tagOfInterest.pose.x-.5, tagOfInterest.pose.y))
                                    .build();
                            drive.followTrajectorySequenceAsync(tag);
                        }

                    }
                    else{
                        switch (blueDetection.getLocation())
                        {
                            case LEFT:
                                drive.followTrajectorySequenceAsync(leftBoard);


                            case RIGHT:
                                drive.followTrajectorySequenceAsync(rightBoard);

                            case MIDDLE:
                                drive.followTrajectorySequenceAsync(leftBoard);

                        }

                    }
                    ID_TAG_OF_INTEREST = STACK;
                    currentState = state.stack;

                    break;
                case stack:
                    TrajectorySequence toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(0,0))
                            .build();
                    drive.followTrajectorySequenceAsync(toStack);
                    currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                    for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == ID_TAG_OF_INTEREST)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                    if(tagFound){
                        TrajectorySequence tag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(tagOfInterest.pose.x-.5, tagOfInterest.pose.y))
                                .build();
                        drive.followTrajectorySequenceAsync(tag);
                    }
                    break;
            }




        }


    }


    private void initCam() {

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

    public void initAprilTagDetection(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

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

}   // end class