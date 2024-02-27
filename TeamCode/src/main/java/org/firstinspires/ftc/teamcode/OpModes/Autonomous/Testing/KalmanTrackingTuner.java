
//package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;
//
//import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.MAX_ACCEL;
//import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.MAX_VEL;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
//
//@Config
//@Autonomous(group = "drive")
//public class KalmanTrackingTuner extends LinearOpMode {
//
//    public static double DISTANCE = 50;
//
//    enum Mode {
//        DRIVER_MODE,
//        TUNING_MODE
//    }
//
//    private KalmanTrackingTuner.Mode mode;
//
//    private static MotionProfile generateProfile(boolean movingForward) {
//        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
//        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
//        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
//    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
//                .forward(DISTANCE)
//                .build();
//
//        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
//                .back(DISTANCE)
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(trajectoryForward);
//            drive.followTrajectory(trajectoryBackward);
//        }
//    }
//}

