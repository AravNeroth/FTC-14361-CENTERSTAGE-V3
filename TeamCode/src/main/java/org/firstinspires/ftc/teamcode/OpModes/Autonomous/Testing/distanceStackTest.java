package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "distanceStackTest", group = "Auto")
public class distanceStackTest extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;
    Robot bot;
    currentState currentState;
    boolean blueAlliance, redAlliance;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31, 61, Math.toRadians(90));

        currentState = currentState.tape;
        blueAlliance = true;

        initCam();

        waitForStart();

        camera.stopStreaming();

        if (isStopRequested()) return;

        telemetry.addLine("Sensor & Drivetrain Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;


        switch(currentState)
        {
//        case tape:
//            drive.setPoseEstimate(startPose);
//            bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
//            bot.setOuttakeSlideState(outtakeSlidesState.STATION);
//            bot.setArmState(armState.intaking);
//            bot.setArmPosition(armState.intaking, armExtensionState.extending);
//            bot.setWristPosition(wristState.intaking);
//            bot.setWristState(wristState.intaking);
//
//            if (blueDetection.getLocation() == HSVBlueDetection.Location.LEFT) {
//                TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
//                        //Initialization
//                        .addDisplacementMarker(() -> {
//                            bot.setLidPosition(lidState.close);
//                        })
//
//                        //Move away from wall
//                        .lineToConstantHeading(new Vector2d(-36, 55))
//
//                        //Move pixel to tape
//                        .lineToLinearHeading(new Pose2d(-37, 32, Math.toRadians(180)))
//                        .lineToConstantHeading(new Vector2d(-33, 32))
//                        .lineToConstantHeading(new Vector2d(-37, 32))
//                        .build();
//
//                drive.followTrajectorySequence(left);
//            } else if (blueDetection.getLocation() == HSVBlueDetection.Location.MIDDLE) {
//                TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
//                        //Initialization
//                        .addDisplacementMarker(() -> {
//                            bot.setLidPosition(lidState.close);
//                        })
//
//                        //Move away from wall
//                        .lineToConstantHeading(new Vector2d(-36, 55))
//                        //Push to tape
//                        .lineToConstantHeading(new Vector2d(-36, 32))
//                        //Move away from tape
//                        .lineToConstantHeading(new Vector2d(-36, 39))
//                        .build();
//
//                drive.followTrajectorySequence(center);
//            } else {
//                TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                        //Initialization
//                        .addDisplacementMarker(() -> {
//                            bot.setLidPosition(lidState.close);
//                        })
//
//                        //Move to tape
//                        .lineToConstantHeading(new Vector2d(-45, 50))
//                        //Push to tape
//                        .lineToConstantHeading(new Vector2d(-45, 42))
//                        //Move away from tape
//                        .lineToConstantHeading(new Vector2d(-45, 47.5))
//                        .build();
//
//                drive.followTrajectorySequence(right);
//            }
//            break;
//        case board:
//            TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
//                    .build();
            case stack1:
                TrajectorySequence detect = drive.trajectorySequenceBuilder(startPose)

                        .build();
        }
    }

//    public void distanceTelemetry(){
//        // if true, then blue. if false, red detect
//        if(distSensor.getAlliance())
//            telemetry.addLine("Using Sensor: LEFT");
//        else
//            telemetry.addLine("Using Sensor: RIGHT");
//
//        telemetry.addData("Distance To Wall: ", distSensor.getDistance());
//    }

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
}




