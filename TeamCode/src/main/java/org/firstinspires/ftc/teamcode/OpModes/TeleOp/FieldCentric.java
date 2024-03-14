package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.holderServoState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.linkageState;
import org.firstinspires.ftc.teamcode.Commands.mecanumState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.slowDownState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class FieldCentric extends OpMode {
    private ElapsedTime runTime;
    private GamepadEx driver, operator;
    private Robot bot;
    boolean startTime = false, endTime = false;
    double startTimeS = 0, endTimes = 0;

    @Override
    public void init() {
        runTime = new ElapsedTime();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        bot = new Robot(hardwareMap, telemetry);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("It's goobin time");
        telemetry.addLine("Time taken: " + getRuntime() + " seconds.");


        telemetry.update();

        bot.setArmPosition(armState.intaking, armExtensionState.extending);

        bot.setHolderServoPosition(holderServoState.open);

        bot.setWristPosition(wristState.intaking);

        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

        bot.setMecanumState(mecanumState.NORMAL);

        bot.setLinkagePosition(linkageState.LOW);

        bot.setLidPosition(lidState.open);

        bot.setDrone();

        bot.setSlowDownState(slowDownState.FULL);
    }

    // ---------------------------- LOOPING ---------------------------- //

    @Override
    public void loop() {
        if(!startTime){
            startTimeS = getRuntime();
            startTime = true;
        }
        telemetry.addLine("Total Start Time" + startTimeS + " seconds.");

        telemetry.addLine("Left Slide Position: " + bot.getOuttakeLeftSlidePosition() + " ticks");
        telemetry.addLine("Right Slide Position: " + bot.getOuttakeRightSlidePosition() + " ticks");
//
//        telemetry.addLine("Left US With Equation" + bot.ultrasonicSensor.getVoltageWEquation());
//        telemetry.addLine("Left US With Equation Average" + bot.ultrasonicSensor.getAverageWEquation(10));
//        telemetry.addLine("Left US Center" + bot.ultrasonicSensor.getLeftDistanceCenter());
//        telemetry.addLine("Left US Strafe" + (30-bot.ultrasonicSensor.getLeftDistanceCenter()));
        double d1 = bot.distanceSensor.getBotsLeftEdgeDistance();
        double d2 = bot.ultrasonicSensor.getLeftDistanceEdge();
        telemetry.addLine("Dis Sensor" + d1);
        telemetry.addLine("Ultrasonic" + d2);



//        telemetry.addLine("US Disance " + bot.distanceSensor.getUSDistance());
//        telemetry.addLine("US Distance " + bot.distanceSensor.getUSDistanceNoDiv());
//
//        telemetry.addLine("US Distance " + bot.distanceSensor.getUSDistance360());
        bot.driveTrain.driveAngleLock(bot.getMecanumState(), driver);
        bot.driveTrain.setMotorPower();

        telemetry.update();

        driver.readButtons();
        operator.readButtons();

        // ---------------------------- DRIVER CODE ---------------------------- //

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            bot.driveTrain.resetIMU();
        }
        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            bot.outtakeSlide.setOuttakeSlidesPID(outtakeSlidesState.HIGHOUT);
        }

//        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
//            bot.setMecanumState(mecanumState.TOBLUEBACKBOARD);
//        }
//        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
//            bot.setMecanumState(mecanumState.TOREDBACKBOARD);
//        }
        if(driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bot.setMecanumState(mecanumState.NORMAL);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.BACK)) {
          bot.setMecanumState(mecanumState.ROBOTCENTRIC);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            if (bot.getActiveIntakeState() != null && (bot.getActiveIntakeState().equals(activeIntakeState.active))) {
                bot.setActiveIntakePosition(activeIntakeState.inactive);
                bot.setActiveIntakeState(activeIntakeState.inactive);
            } else if ((bot.getOuttakeState().equals(outtakeSlidesState.STATION) && (bot.getArmState()).equals(armState.intaking) && (bot.getWristState()).equals(wristState.intaking))) {
                bot.setLidPosition(lidState.open);
                bot.setHolderServoPosition(holderServoState.open);
                bot.setActiveIntakePosition(activeIntakeState.active);
                bot.setActiveIntakeState(activeIntakeState.active);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            if (bot.getActiveIntakeState() != null && bot.getActiveIntakeState().equals(activeIntakeState.activeReverse)) {
                bot.setActiveIntakePosition(activeIntakeState.inactive);
                bot.setActiveIntakeState(activeIntakeState.inactive);
            } else {
                bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                bot.setActiveIntakeState(activeIntakeState.activeReverse);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            if (bot.getLidState() != null && bot.getLidState().equals(lidState.close) && bot.getHolderState() != null && bot.getHolderState().equals(holderServoState.close)) {
                bot.setLidPosition(lidState.open);
                bot.setHolderServoPosition(holderServoState.open);
            }
            else
            {
                bot.setLidPosition(lidState.close);
                bot.setHolderServoPosition(holderServoState.close);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            if(bot.getLidState() != null && bot.getLidState().equals(lidState.close) && bot.getHolderState() != null && bot.getHolderState().equals(holderServoState.close))
            {
                bot.setLidPosition(lidState.open);
            }
            else if(bot.getLidState() != null && bot.getLidState().equals(lidState.open) && bot.getHolderState() != null && bot.getHolderState().equals(holderServoState.close))
            {
                bot.setHolderServoPosition(holderServoState.open);
            }
            else
            {
                bot.setLidPosition(lidState.close);
                bot.setHolderServoPosition(holderServoState.close);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.launchDrone();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (bot.getLinkageState() != null && bot.getLinkageState().equals(linkageState.HIGH)) {
                bot.setLinkagePosition(linkageState.LOW);
            } else if (bot.getLinkageState() != null && bot.getLinkageState().equals(linkageState.LOW)) {
                bot.setLinkagePosition(linkageState.HIGH);
            } else {
                bot.setLinkagePosition(linkageState.LOW);
            }

        }

        // --------------------------- OPERATOR CODE --------------------------- //


        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            if (bot.getWristState() != null && bot.getWristState().equals(wristState.intaking)) {
                bot.setWristPosition(wristState.outtaking);
                bot.setWristState(wristState.outtaking);
            } else if (bot.getWristState() != null && bot.getWristState().equals(wristState.init)) {
                bot.setWristPosition(wristState.outtaking);
                bot.setWristState(wristState.outtaking);
            } else {
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
            }
        }
        if (operator.wasJustPressed(GamepadKeys.Button.BACK)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.HIGHOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.HIGHOUT);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.MEDIUMOUT);
        }

        if(operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            bot.setLidPosition(lidState.close);
            bot.setHolderServoPosition(holderServoState.close);
        }

        if(operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            bot.setWristPosition(wristState.init);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.LOWMED, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.LOWMED);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.LOWOUT);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
            bot.setWristState(wristState.outtaking);
            bot.setWristPosition(wristState.outtaking);

            bot.setArmPosition(armState.outtaking, armExtensionState.extending);
            bot.setArmState(armState.outtaking);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {


            bot.setWristState(wristState.intaking);
            bot.setWristPosition(wristState.intaking);
            bot.setLidPosition(lidState.open);

            bot.setArmPosition(armState.intaking, armExtensionState.extending);
            bot.setArmState(armState.intaking);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.X)) {

            if (bot.getArmState() != null && bot.getArmState().equals(armState.outtaking)) {
                bot.setWristPosition(wristState.intaking);
            }
            if (bot.getArmState() != null && bot.getArmState().equals(armState.intaking)) {
                bot.setArmPosition(armState.init, armExtensionState.extending);
                bot.setArmPosition(armState.init, armExtensionState.extending);
                bot.setLidPosition(lidState.open);

            }
            bot.setArmPosition(armState.init, armExtensionState.extending);
            bot.setArmState(armState.init);
        }

        if (operator.getRightY() > .1) {

            bot.outtakeSlide.setLeftOuttakeSlidePosition((int) bot.outtakeSlide.getLeftOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 80));
            bot.outtakeSlide.setRightouttakeSlidePosition((int) bot.outtakeSlide.getRightOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 80));
        }
        if (operator.getRightY() < -.1) {

            bot.outtakeSlide.setLeftOuttakeSlidePosition((int) bot.outtakeSlide.getLeftOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 80));
            bot.outtakeSlide.setRightouttakeSlidePosition((int) bot.outtakeSlide.getRightOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 80));
        }
//        if(operator.getLeftY() > .1){
//
//        }
        if(operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
          bot.outtakeSlide.resetSlideEncoder();
        }
        if(!endTime){
            endTimes = getRuntime();
            endTime = true;
            telemetry.addLine("end time"+ endTimes);
            telemetry.update();
        }

        telemetry.addLine("end time"+ endTimes);
        telemetry.update();
    }
}