package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.Subsystems.RobotPID;

@Config
@Autonomous(group = "drive")
public class OuttakePIDTuner extends OpMode {
    private GamepadEx operator;
    private RobotPID bot;
    private PIDController controller;
    private final double ticks_in_degree = 384.539792388;

    public static int target = 1000;
    private int previous_target = 0;

    public static double kcos = 0.1;

    public static double max_v = 8000;
    public static double max_a = 8000;

    public static double p, i, d;
    private double pp, pi, pd;
    private double leftPid, rightPid;
    public VoltageSensor voltageSensor;

    private ElapsedTime time;

    private MotionProfile profile;

    @Override
    public void init() {
        operator = new GamepadEx(gamepad2);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new RobotPID(hardwareMap, telemetry);
        time = new ElapsedTime();

        p = 0;
        d = 0;

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
            controller.setPID(p, i, d);
        }

        if (target != previous_target) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(target, 0), max_v, max_a);

            time.reset();
            previous_target = target;
        }

        pp = p;
        pi = i;
        pd = d;

        double leftSlidePose = bot.getOuttakeLeftSlidePosition();
        double rightSlidePose = bot.getOuttakeRightSlidePosition();

        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double target = targetState.getX();
        leftPid = controller.calculate(leftSlidePose, target);
        rightPid = controller.calculate(rightSlidePose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kcos;

        double leftPower = (leftPid + ff) / voltageSensor.getVoltage() * 12.0;
        double rightPower = (rightPid + ff) / voltageSensor.getVoltage() * 12.0;

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        {
            bot.outtakeSlide.setPosition((int) target, leftPid, rightPid);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
        {
            bot.outtakeSlide.setPosition(0, leftPid, rightPid);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
        {
            bot.outtakeSlide.setLeftPower(0);
            bot.outtakeSlide.setRightPower(0);
            bot.setOuttakeSlideState(outtakeSlidesState.IDLE);
        }

        telemetry.addData("Left Slide Pose : ", leftSlidePose);
        telemetry.addData("Right Slide Pose : ", rightSlidePose);
        telemetry.addData("Target Pose: ", target);
        telemetry.addData("Left Controller PID: ", leftPid);
        telemetry.addData("Right Controller PID: ", rightPid);
        telemetry.addData("ff : ", ff);
        telemetry.addData("Left Power: ", leftPower);
        telemetry.addData("Right Power: ", rightPower);
        telemetry.update();
    }
}