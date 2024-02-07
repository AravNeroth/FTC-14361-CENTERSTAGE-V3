package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class AutoTest {
    @Autonomous(name = "closeBlueObjectDetect", group = "Auto")
    public class closeBlue extends LinearOpMode {
        Robot bot;

        public void runOpMode() {
            bot = new Robot(hardwareMap, telemetry);
            Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0.0, 0.0), Math.toRadians(90)));

            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    MecanumDrive.FollowTrajectoryAction(Math.PI / 2),
                    new ParallelAction(
                            drive.followTrajectory(shootingTraj),
                            new SequentialAction(
                                    shooter.spinUp(),
                                    shooter.fireBall()
                            )
                    )
            ));


        }
    }
}
