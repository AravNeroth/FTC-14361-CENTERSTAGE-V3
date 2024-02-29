package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.holderServoState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.linkageState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.slowDownState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.mecanumState;

public class RobotPID {
    public OuttakeSlidePID outtakeSlide;
    public Mecanum driveTrain;
    public Wrist wrist;
    public Arm arm;
    public Drone drone;
    public outtakeSlidesState outtakeSlidesState;
    public wristState wristState;
    public armState armState;
    public armExtensionState armExtensionState;
    public extensionState extensionState;
    public mecanumState mecanumState;
    public distanceSensor distanceSensor;
    public Lid lid;
    public lidState lidstate;
    public colorSensor colorSensor;
    public ActiveIntake activeIntake;
    public activeIntakeState activeIntakeState;
    public slowDownState slowDownState;
    public Linkage linkage;
    public linkageState linkageState;
    public VoltMecanum voltMecanum;
    public holderServo holderServo;
    public holderServoState holderServoState;
    Telemetry telemetry;

    public RobotPID(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        driveTrain = new Mecanum(hardwareMap);
        linkage = new Linkage(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        outtakeSlide = new OuttakeSlidePID(hardwareMap);
        activeIntake = new ActiveIntake(hardwareMap);
        lid = new Lid(hardwareMap);
        drone = new Drone(hardwareMap);
        holderServo = new holderServo(hardwareMap);
        voltMecanum = new VoltMecanum(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);
    }

    public void setPid(double p, double i, double d)
    {
        outtakeSlide.setPid(p,i,d);
    }

    // ---------------------------- OuttakeSlide ---------------------------- //

    public void setOuttakeSlidePosition(outtakeSlidesState outtakeSlidesState, extensionState extensionState) {
        outtakeSlide.setOuttakeSlidePosition(extensionState, outtakeSlidesState);
        this.outtakeSlidesState = outtakeSlidesState;
    }

    public outtakeSlidesState getOuttakeState() {
        return outtakeSlidesState;
    }

    public void setOuttakeSlideState(outtakeSlidesState outtakeSlidesState) {
        this.outtakeSlidesState = outtakeSlidesState;
    }

    public double getOuttakeLeftSlidePosition() {
        return outtakeSlide.getLeftOuttakeSlideMotorPosition();
    }

    public double getOuttakeRightSlidePosition() {
        return outtakeSlide.getRightOuttakeSlideMotorPosition();
    }

    // ---------------------------- Wrist ---------------------------- //

    public void setWristPosition(wristState wristState) {
        wrist.setWristPosition(wristState);
        this.wristState = wristState;
    }

    public void setWristState(wristState wristState) {
        this.wristState = wristState;
    }

    public wristState getWristState() {
        return wristState;
    }

    public double getWristPosition() {
        return wrist.getWristPosition();
    }

    // ---------------------------- Arm ---------------------------- //

    public void setArmPosition(armState armState, armExtensionState armExtensionState) {
        arm.setArmPosition(armState, armExtensionState);
        this.armState = armState;
    }

    public void setArmState(armState armState) {
        this.armState = armState;
    }

    public armState getArmState() {
        return armState;
    }

    public armExtensionState getArmExtensionState() {
        return armExtensionState;
    }

    public void setArmExtensionState(armExtensionState armExtensionState) {
        this.armExtensionState = armExtensionState;
    }

    // ---------------------------- ActiveIntake ---------------------------- //

    public void setActiveIntakePosition(activeIntakeState activeIntakeState) {
        activeIntake.setActiveIntakePosition(activeIntakeState);
        this.activeIntakeState = activeIntakeState;
    }

    public activeIntakeState getActiveIntakeState() {
        return activeIntakeState;
    }

    public void setActiveIntakeState(activeIntakeState activeIntakeState) {
        this.activeIntakeState = activeIntakeState;
    }

    // ---------------------------- SlowDown ---------------------------- //

    public slowDownState getSlowDownState() {
        return slowDownState;
    }

    public void setSlowDownState(slowDownState slowDownState) {
        this.slowDownState = slowDownState;
    }

    // ---------------------------- Drone ---------------------------- //

    public void setDrone() {
        drone.resetDrone();
    }

    public void launchDrone() {
        drone.launch();
    }

    // ---------------------------- Linkage ---------------------------- //

    public void setLinkagePosition(linkageState linkageState) {
        linkage.setLinkagePosition(linkageState);
        this.linkageState = linkageState;
    }

    public void setLinkageState(linkageState linkageState) {
        this.linkageState = linkageState;
    }

    public linkageState getLinkageState() {
        return this.linkageState;
    }

    public void getLinkagePosition() {
        linkage.getLinkagePosition();
    }

    //---------------------------- Lid ---------------------------- //
    public void setLidState(lidState lidstate) {

        this.lidstate = lidstate;
    }

    public void setLidPosition(lidState lidstate) {
        lid.setLidPosition(lidstate);
        this.lidstate = lidstate;
    }

    public lidState getLidState() {
        return lidstate;
    }

    public void setLidCustomPosition(double position) {
        lid.setLidCustomPosition(position);
    }

    //---------------------------- Holder Servo ---------------------------- //
    public void setHolderServoState(holderServoState holderState) {

        this.holderServoState = holderState;
    }

    public void setHolderServoPosition(holderServoState holderState) {
        holderServo.setHolderServoPosition(holderState);
        this.holderServoState = holderState;
    }

    public holderServoState getHolderState() {
        return holderServoState;
    }

    public void setHolderServoCustomPosition(double position) {
        holderServo.setHolderServoCustomPosition(position);
    }

    //---------------------------- Mecanum ---------------------------- //
    public void setMecanumState(mecanumState mecanumState){
        this.mecanumState = mecanumState;
    }
    public mecanumState getMecanumState(){
        return mecanumState;
    }
    public void runMecanum(mecanumState mecanumState, GamepadEx gamepad1){
        driveTrain.driveAngleLock(mecanumState, gamepad1);
    }
}