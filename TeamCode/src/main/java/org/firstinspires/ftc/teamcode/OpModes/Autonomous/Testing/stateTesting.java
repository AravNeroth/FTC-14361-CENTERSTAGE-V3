package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous(name = "StateTesting ", group = "test")
public class stateTesting extends LinearOpMode {
    boolean tagFound = false;
    boolean caseTagFound = false;
    String webcamName;
    Robot robot;
    ElapsedTime intake = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY, stack1Y, stackDetectX, stackDetectY;
    //   aprilTagDetection aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    double leftTapeX, leftTapeY, centerTapeX, centerTapeY, rightTapeX, rightTapeY;
    HSVBlueDetection blueDetection;
    int location = 4;
    Pose2d currentPose;
    Pose2d boardPose;

    AprilTagDetection tagOfInterest = null;
    state currentState = state.tape;
    double boardXOffset, pathXoffset;
    double boardYOffset = 0;
    boolean pixelDropped = false;
    boolean intaking = false;

    enum state {
        tape, board1,board2,board3,stack
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
                        case RIGHT:
                            drive.followTrajectorySequenceAsync(rightTape);
                        case MIDDLE:
                            drive.followTrajectorySequenceAsync(centerTape);
                    }
                    currentState = state.board1;
                    break;
                case board1:
                    

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
}




