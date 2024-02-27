package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotPID;
import org.firstinspires.ftc.teamcode.util.robotConstants;

@Config
@Autonomous(group = "drive")
public class OuttakeSlidePIDTest extends LinearOpMode {
    public FtcDashboard dashboard;
    private ElapsedTime runTime;
    private GamepadEx driver, operator;
    private RobotPID bot;
    public PIDController pidController;

    public double leftSlidePosition, rightSlidePosition;

    public double leftPidValue = 0;
    public double rightPidValue = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static int DISTANCE = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        runTime = new ElapsedTime();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        bot = new RobotPID(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();

        leftPidValue = 0;
        rightPidValue = 0;

        kP = 0;
        kI = 0;
        kD = 0;

        pidController = new PIDController(kP, kI, kD);

        telemetry.addLine("PID Testing");

        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addLine("P: " + kP);
            telemetry.addLine("I: " + kI);
            telemetry.addLine("D: " + kD);

            telemetry.update();

            driver.readButtons();
            operator.readButtons();

            bot.driveTrain.setMotorPower();

            pidController.setPID(kP, kI, kD);

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                leftSlidePosition = bot.getOuttakeLeftSlidePosition();
                rightSlidePosition = bot.getOuttakeRightSlidePosition();

                leftPidValue = pidController.calculate(leftSlidePosition, DISTANCE);
                rightPidValue = pidController.calculate(rightSlidePosition, DISTANCE);

                bot.outtakeSlide.setPosition(DISTANCE, leftPidValue, rightPidValue);
            }

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            {
                leftSlidePosition = bot.getOuttakeLeftSlidePosition();
                rightSlidePosition = bot.getOuttakeRightSlidePosition();

                leftPidValue = pidController.calculate(leftSlidePosition, 0);
                rightPidValue = pidController.calculate(rightSlidePosition, 0);

                bot.outtakeSlide.setPosition(0, leftPidValue, rightPidValue);
            }
        }
    }
}
