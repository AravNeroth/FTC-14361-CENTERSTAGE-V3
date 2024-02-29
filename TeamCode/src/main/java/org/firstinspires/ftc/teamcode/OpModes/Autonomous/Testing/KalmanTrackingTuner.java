
package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.opmode.ManualFeedforwardTuner;

import java.util.Objects;

import ftc.rogue.blacksmith.util.kalman.KalmanFilter;

@Config
@Autonomous(group = "drive")
public class KalmanTrackingTuner extends LinearOpMode {

    public static double DISTANCE = 50;
    public static double headingFilterR, headingFilterQ, headingVelocityFilterR, headingVelocityFilterQ, pos1VelocityFilterR, pos1VelocityFilterQ, pos2VelocityFilterR, pos2VelocityFilterQ, pos1FilterR, pos1FilterQ, pos2FilterR, pos2FilterQ;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private KalmanTrackingTuner.Mode mode;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
            .forward(DISTANCE)
            .build();

    Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
            .back(DISTANCE)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    waitForStart();

                    while (opModeIsActive() && !isStopRequested()) {
                        drive.followTrajectory(trajectoryForward);
                        drive.followTrajectory(trajectoryBackward);
                    }

                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }
        }
    }
}