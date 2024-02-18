package org.firstinspires.ftc.teamcode.OpModes.Autonomous.FullPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection.NewVision;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "closeRedObjectDetect", group = "Auto")
public class closeRed extends LinearOpMode {
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
    Pose2d start = new Pose2d(-36.5, -62.75, Math.toRadians(270));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    boolean cameraOn = false, aprilTagOn = false, toAprilTag1 = false, initCam = false, randomTag = false, underTrussBool = false, stackBool = false, finishBoard = false;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    double centerTagX = 60.275, centerTagY = -29.4;
    boolean onePixel = false, twoPixels = false;
    double tagY = 0;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double tagsize = 0.166;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int LEFT = 4, MIDDLE = 5, RIGHT = 6, REDSTACK = 7;
    int ID_TAG_OF_INTEREST = 4;
    double batteryOffset = 0;
    boolean tagFound = false;

    double leftTapeX = 0, leftTapeY = 0, centerTapeX = 11.5, centerTapeY = -34.5, rightTapeX = 0, rightTapeY = 0;
    double leftBoardX, leftBoardY, rightBoardX, rightBoardY;
    double secondTimeBoardX = 0, secondTimeBoardY = 0, thirdTimeBoardX, thirdTimeBoardY;



    enum state {
        tape, underTruss,firstTimeBoard, secondTimeBoard, thirdTimeBoard, toStack, idle,leaveStack
    }
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -62.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))

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
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))

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
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 45, DriveConstants.TRACK_WIDTH))
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

        switch (newVision.getStartingPosition())
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
            case CENTER:
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
    private void newColorDetect(){


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

            //  NewVision.StartingPosition startingPos = NewVision.StartingPosition.LEFT;
            visionPortal.resumeStreaming();
//            telemetry.addLine("vision portal built");
//            telemetry.addData("starting position: ", startingPos);
//            startingPos = newVision.getStartingPosition();
            //     telemetry.addData("called NewVision- returned: ", startingPos);
            initCam = true;
        }


    }
