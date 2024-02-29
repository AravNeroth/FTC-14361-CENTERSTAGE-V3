package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.Subsystems.RobotPID;

@Config
@Autonomous(group = "drive")
public class OuttakePIDTuner extends OpMode {
    private GamepadEx operator;
    private RobotPID bot;
    private PIDFController controller;
    private final double ticks_in_degree = 384.5/360;

    public static int DISTANCE = 1000;
    private int previous_target = 0;

    public static double kcos = 0.1;

    public static double max_v = 8000;
    public static double max_a = 8000;

    public static double p, i, d, f;
    private double pp, pi, pd;
    private double leftPid, rightPid;

    private ElapsedTime time;

    private MotionProfile profile;

    @Override
    public void init() {
        operator = new GamepadEx(gamepad2);
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new RobotPID(hardwareMap, telemetry);
        time = new ElapsedTime();

        bot.setWristPosition(wristState.init);
        bot.setArmPosition(armState.init, armExtensionState.extending);

        p = 0;
        i = 0;
        d = 0;
        f = 0;

        telemetry.addLine("PID Testing");
        telemetry.addLine("Ready!");
        telemetry.addLine("Starting with Idle ff tuning");
        telemetry.update();
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        operator.readButtons();

        bot.driveTrain.setMotorPower();

        if (p != pp || i != pi || d != pd) {
            controller.setPIDF(p, i, d, f);
        }

        if (DISTANCE != previous_target) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(DISTANCE, 0), max_v, max_a);

            time.reset();
            previous_target = DISTANCE;
        }

        pp = p;
        pi = i;
        pd = d;

        double leftSlidePose = bot.getOuttakeLeftSlidePosition();
        double rightSlidePose = bot.getOuttakeRightSlidePosition();

        double avgPose = (leftSlidePose + rightSlidePose)/2;

        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
//        double target = targetState.getX();

        leftPid = controller.calculate(avgPose, DISTANCE);
        rightPid = controller.calculate(avgPose, DISTANCE);
        double ff = Math.cos(Math.toRadians(DISTANCE / ticks_in_degree)) * f;

        double leftPower = (leftPid + ff);
        double rightPower = (rightPid + ff);

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        {
            bot.outtakeSlide.setPosition(DISTANCE, leftPower, rightPower);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
        {
            bot.outtakeSlide.setPosition(0, leftPower, rightPower);
        }

        telemetry.addData("Left Slide Pose : ", leftSlidePose);
        telemetry.addData("Right Slide Pose : ", rightSlidePose);
        telemetry.addData("Target Pose: ", DISTANCE);
        telemetry.update();
    }
}