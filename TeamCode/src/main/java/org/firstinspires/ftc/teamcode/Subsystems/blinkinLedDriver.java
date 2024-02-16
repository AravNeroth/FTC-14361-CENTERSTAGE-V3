package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class blinkinLedDriver {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private final static int LED_PERIOD = 10;

    Telemetry.Item patternName;
    Telemetry.Item display;
    DisplayKind displayKind;
    Deadline ledCycleDeadline;
    public blinkinLedDriver(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    public void initBlinkin()
    {
        displayKind = DisplayKind.AUTO;

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);


    }
    public void setPatternYellow(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
    public void setPatternGreen(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public void setPatternPurple(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }
    public void setPatternWhite(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
    public void setPatternRainbowOcean(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }
}
