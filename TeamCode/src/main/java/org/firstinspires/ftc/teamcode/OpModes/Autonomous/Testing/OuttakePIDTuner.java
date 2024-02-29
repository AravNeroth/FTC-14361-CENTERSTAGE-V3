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
    private PIDController controller;
    private final double ticks_in_degree = 384.5/360;

    public static int target = 1000;
    public double Target;
    private int previous_target = 0;

    public static double kcos = 0.1;

    public static double max_v = 10000;
    public static double max_a = 6000;

    public static double p, i, d, f;
    private double pp, pi, pd;
    private double power;
    public double voltage;
    private ElapsedTime voltageTimer, time;

    private MotionProfile profile;
    public MotionState targetState;

    public VoltageSensor voltageSensor;

    @Override
    public void init() {
        operator = new GamepadEx(gamepad2);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new RobotPID(hardwareMap, telemetry);
        voltageTimer = new ElapsedTime();
        time = new ElapsedTime();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        bot.setWristPosition(wristState.init);
        bot.setArmPosition(armState.init, armExtensionState.extending);

        bot.outtakeSlide.setLeftPower(0);
        bot.outtakeSlide.setRightPower(0);

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
        targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        Target = targetState.getX();

        if(voltageTimer.seconds() > 5)
        {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        operator.readButtons();

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

        double avgPose = (leftSlidePose + rightSlidePose)/2;

        double pid = controller.calculate(avgPose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = (pid + ff) / (voltage * 12.0);

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        {
            bot.outtakeSlide.setPosition(power, power);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
        {
            bot.outtakeSlide.setPosition(power, power);
        }

        telemetry.addData("Left Slide Pose : ", leftSlidePose);
        telemetry.addData("Right Slide Pose : ", rightSlidePose);
        telemetry.addData("Target Pose: ", target);
        telemetry.update();
    }

    public double power()
    {
        double leftSlidePose = bot.getOuttakeLeftSlidePosition();
        double rightSlidePose = bot.getOuttakeRightSlidePosition();

        double avgPose = (leftSlidePose + rightSlidePose)/2;

        double pid = controller.calculate(avgPose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = (pid + ff) / (voltage * 12.0);

        return power;
    }

}