package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class OuttakeSlide
{
    DcMotorEx rightouttakeSlide, leftouttakeSlide;
    private final int countsPerRev = 384;

    double power = .9;
    double integralSum = 0;
    double lastError = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double leftPower = 0, rightPower = 0;
    ElapsedTime timer = new ElapsedTime();


    public OuttakeSlide(HardwareMap hardwareMap) {
        rightouttakeSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");
        leftouttakeSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");

        rightouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        rightouttakeSlide.setTargetPositionTolerance(5);
        leftouttakeSlide.setTargetPositionTolerance(5);
    }

    public OuttakeSlide(HardwareMap hardwareMap, boolean PID) {
        rightouttakeSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");
        leftouttakeSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");

       rightouttakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftouttakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftouttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightouttakeSlide.setTargetPositionTolerance(5);
        leftouttakeSlide.setTargetPositionTolerance(5);
    }
    public void setOuttakeSlidePosition(extensionState extensionState, outtakeSlidesState outtakeSlidesState)
    {
        switch(extensionState)
        {
            case retracted:
                break;
            case extending:
            {
                switch (outtakeSlidesState)
                {
                    case MOSTHIGHOUT:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case HIGHOUT:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case MEDIUMOUT:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case LOWMED:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case LOWOUT:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case AUTOLOWOUT:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(power);
                        rightouttakeSlide.setPower(power);
                        break;
                    case STATION:
                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDRIGHT);

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
    public void setOuttakeSlidesPID(outtakeSlidesState outtakeSlidesState)
    {

                switch (outtakeSlidesState)
                {
                    case MOSTHIGHOUT:
                     leftPower = PIDControl(robotConstants.outtakeSlide.MOSTHIGHLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.MOSTHIGHRIGHT, rightouttakeSlide.getCurrentPosition());


                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case HIGHOUT:
                         leftPower = PIDControl(robotConstants.outtakeSlide.HIGHLEFT, leftouttakeSlide.getCurrentPosition());
                       rightPower = PIDControl(robotConstants.outtakeSlide.HIGHRIGHT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case MEDIUMOUT:
                        leftPower = PIDControl(robotConstants.outtakeSlide.MEDIUMLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.MEDIUMRIGHT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case LOWMED:
                        leftPower = PIDControl(robotConstants.outtakeSlide.LOWMEDLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.LOWMEDRIGHT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case LOWOUT:
                        leftPower = PIDControl(robotConstants.outtakeSlide.LOWLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.LOWRIGHT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                    case AUTOLOWOUT:
                        leftPower = PIDControl(robotConstants.outtakeSlide.AUTOLOWLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.AUTOLOWRIGHT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case STATION:
                        leftPower = PIDControl(robotConstants.outtakeSlide.GROUNDLEFT, leftouttakeSlide.getCurrentPosition());
                        rightPower = PIDControl(robotConstants.outtakeSlide.GROUNDLEFT, rightouttakeSlide.getCurrentPosition());

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
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
        leftouttakeSlide.setTargetPosition(pos);
        rightouttakeSlide.setTargetPosition(pos);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(power);
        rightouttakeSlide.setPower(power);
    }
    public void setLeftOuttakeSlidePosition(int pos) {
        leftouttakeSlide.setTargetPosition(pos);


        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftouttakeSlide.setPower(power);

    }
    public void setRightouttakeSlidePosition(int pos) {

        rightouttakeSlide.setTargetPosition(pos);


        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightouttakeSlide.setPower(power);
    }
//    public double PIDControl(double reference, double state){
//double
//    }
    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
//    public void setZero(){
//        leftouttakeSlide.setC
//    }
}

