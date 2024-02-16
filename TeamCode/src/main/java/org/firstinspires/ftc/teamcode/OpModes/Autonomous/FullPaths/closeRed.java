package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "closeRedObjectDetect", group = "Auto")
public class closeRed extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -62.5, Math.toRadians(270));
        initCam();
        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .lineToLinearHeading(new Pose2d(17,-33, Math.toRadians(0)))

//                .lineToConstantHeading(new Vector2d(4, 32))
//                .lineToConstantHeading(new Vector2d(8, 32))
//                .addDisplacementMarker( () -> {
//
//                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
//                    bot.setArmState(armState.autoDrop);
//
//                })
                .lineToConstantHeading(new Vector2d(12,-33))

//                .lineToLinearHeading(new Pose2d(20,30, Math.toRadians(90)))
                //    .lineToConstantHeading(new Vector2d(30,36))
                .lineToConstantHeading(new Vector2d(15,-33))
                .lineToLinearHeading(new Pose2d(30,-28.5, Math.toRadians(180)))
                .addDisplacementMarker( () -> {
                    bot.outtakeSlide.setPosition(500);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                //    .lineToConstantHeading(new Vector2d(21, 32))
                .lineToConstantHeading(new Vector2d(52,-29))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(51.8,-29))
                .addDisplacementMarker(() -> {
                    bot.outtakeSlide.setPosition(730);
                })

                .resetVelConstraint()
                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(40,-29))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToLinearHeading(new Pose2d(40 ,-52.5, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(52,-52.5))
                .build();

        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))

                .lineToConstantHeading(new Vector2d(14,-55))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })

                .lineToConstantHeading(new Vector2d(14, -34.5))

                .lineToConstantHeading(new Vector2d(14, -38))
                .lineToLinearHeading(new Pose2d(40 ,-34, Math.toRadians(180)))
                .addDisplacementMarker( () -> {
                    bot.outtakeSlide.setPosition(500);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(52,-32))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(.25)


                .lineToConstantHeading(new Vector2d(51.8, -32))
                .addDisplacementMarker( () -> {
                    bot.outtakeSlide.setPosition(730);
                })

                .resetVelConstraint()
                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(40, -32))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40, -47))
                .lineToLinearHeading(new Pose2d(40 ,-58, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(52,-54.5))

                .build();


        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 45, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(22.5,-39))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })



                .lineToConstantHeading(new Vector2d(22.25,-50))
                .addDisplacementMarker( () -> {
                    bot.outtakeSlide.setPosition(500);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                .lineToLinearHeading(new Pose2d(48 ,-35, Math.toRadians(180)))


                .lineToConstantHeading(new Vector2d(51,-39))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(50.8,-39))
                .addDisplacementMarker( () -> {
                    bot.outtakeSlide.setPosition(730);
                })

                .resetVelConstraint()
                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(40, -42.5))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToLinearHeading(new Pose2d(40, -53, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(52, -54.5))


                .build();


        //camera initialization -----------------------------------------------------
        // initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (redDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);


                drive.followTrajectorySequence(left);

                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);


                drive.followTrajectorySequence(right);

                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);


                drive.followTrajectorySequence(center);

                break;
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
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
}