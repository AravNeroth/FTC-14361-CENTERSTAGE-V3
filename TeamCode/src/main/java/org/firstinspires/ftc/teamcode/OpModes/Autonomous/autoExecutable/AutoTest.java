package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
//         ----------------------------------------------
//         still wondering wtf action is but hopefully this works !
            Action okipullup = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(42, -36, 0), 0)
                    .build();


            Actions.runBlocking(okipullup);
//         ----------------------------------------------
            /*
            close left blue test! i put in some random ahh values
            does this even make sense chat what teh hell is this new uodate
             */
//            Actions.runBlocking(new SequentialAction(
//                            // Red Right Purple Right
//                            drive.actionBuilder(new Pose2d(13, -60, Math.toRadians(90)))
//                                    .splineTo(new Vector2d(13, -46), Math.toRadians(90))
//                                    .splineTo(new Vector2d(18, -38), Math.toRadians(60))
//                                    .build()
//                    )
//            );
//          ---------------------------------------------
//             Traj Action Builder test
//            TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose)
//                    .strafeTo(new Vector2d(-42, -63))
//                    .turn(0.00001)
//                    .lineToY(-35);

        }
    }
}
