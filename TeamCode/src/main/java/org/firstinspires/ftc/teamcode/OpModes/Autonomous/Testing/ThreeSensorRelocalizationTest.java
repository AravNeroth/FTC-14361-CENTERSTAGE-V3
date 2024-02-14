package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Subsystems.StackColorSensors;

@Autonomous(name = "3 Sensor Relocalization Test", group = "goobTest")

public class ThreeSensorRelocalizationTest extends LinearOpMode {

    private DistanceSensor distFront;
    private StackColorSensors colorSensors;

    @Override
    public void runOpMode() throws InterruptedException {

        distFront = new DistanceSensor(hardwareMap);
        colorSensors = new StackColorSensors(hardwareMap);

        telemetry.addLine("All 3 Sensors Initialized.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            threeSensorTelem();
            telemetry.update();
        }
    }

    public void threeSensorTelem(){
        telemetry.addLine("Left Sensor Values Red : " + colorSensors.leftColor.red() + " Green: " + colorSensors.leftColor.green() + " Blue: " + colorSensors.leftColor.blue());
        telemetry.addLine("Right Sensor Values Red : " + colorSensors.rightColor.red() + " Green: " + colorSensors.rightColor.green() + " Blue: " + colorSensors.rightColor.blue());
        telemetry.addData("Distance Sensor Reading: ", distFront.getDistance());
        checkStackVisionLeft();
        checkStackVisionRight();
    }
    public void checkStackVisionLeft(){
        if(colorSensors.getlColor())
            telemetry.addLine("Left Sensor Detecting Stack!");
    }

    public void checkStackVisionRight(){
        if(colorSensors.getrColor())
            telemetry.addLine("Right Sensor Detecting Stack!");
    }
}
