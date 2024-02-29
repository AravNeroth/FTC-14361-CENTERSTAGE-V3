package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class OuttakeSlidePID
{
    DcMotorEx rightouttakeSlide, leftouttakeSlide;
    double leftPower, rightPower, basePower, leftSlidePosition, rightSlidePosition;
    public PIDController controller;
    public VoltageSensor voltageSensor;
    private final double ticks_in_degree = 384.5/360;
    double pid, ff, f, power;


    public OuttakeSlidePID(HardwareMap hardwareMap) {
        rightouttakeSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");
        leftouttakeSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");

        rightouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        basePower = 1;

        f = 0.0;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        rightouttakeSlide.setTargetPositionTolerance(5);
        leftouttakeSlide.setTargetPositionTolerance(5);
    }

    public void setOuttakeSlidePosition(extensionState extensionState, outtakeSlidesState outtakeSlidesState)
    {
        switch(extensionState)
        {
            case extending:
            {
                switch (outtakeSlidesState)
                {
                    case MOSTHIGHOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.MOSTHIGHLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);

                        break;
                    case HIGHOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.HIGHLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.HIGHLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case MEDIUMOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.MEDIUMLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.MEDIUMLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case LOWMED:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.LOWMEDLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWMEDLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case LOWOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.LOWLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case AUTOLOWOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.AUTOLOWLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.AUTOLOWLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case STATION:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDLEFT);

                        pid = controller.calculate(leftSlidePosition, robotConstants.outtakeSlide.GROUNDLEFT);
                        ff = Math.cos(Math.toRadians(robotConstants.outtakeSlide.GROUNDLEFT / ticks_in_degree)) * f;
                        power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                }
            }

            case extended:
                break;
        }
    }
    public double getLeftOuttakeSlideMotorPosition()
    {
        return leftouttakeSlide.getCurrentPosition();
    }

    public double getRightOuttakeSlideMotorPosition()
    {
        return rightouttakeSlide.getCurrentPosition();
    }
    public void setPosition(int pos) {
        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
        rightSlidePosition = getRightOuttakeSlideMotorPosition();

        leftouttakeSlide.setTargetPosition(pos);
        leftouttakeSlide.setTargetPosition(pos);

        double pid = controller.calculate(leftSlidePosition, pos);
        double ff = Math.cos(Math.toRadians(pos / ticks_in_degree)) * f;
        double power = (pid + ff) * 12.0/ (voltageSensor.getVoltage());

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(power);
        rightouttakeSlide.setPower(power);
    }

    public void setPos(int target, double power)
    {
        leftouttakeSlide.setTargetPosition(target);
        leftouttakeSlide.setTargetPosition(target);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(power);
        rightouttakeSlide.setPower(power);
    }

    public void setLeftOuttakeSlidePosition(int pos) {
        leftouttakeSlide.setTargetPosition(pos);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(basePower);

    }
    public void setRightOuttakeSlidePosition(int pos) {

        rightouttakeSlide.setTargetPosition(pos);


        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightouttakeSlide.setPower(basePower);
    }

    public void setPid(double p, double i, double d)
    {
        controller.setPID(p,i,d);
    }

    public void setLeftPower(double power)
    {
        leftPower = power;
    }

    public void setRightPower(double power)
    {
        rightPower = power;
    }
}