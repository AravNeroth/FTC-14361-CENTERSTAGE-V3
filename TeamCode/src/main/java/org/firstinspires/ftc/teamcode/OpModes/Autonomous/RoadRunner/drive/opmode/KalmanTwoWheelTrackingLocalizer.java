package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.opmode;
import com.acmerobotics.roadrunner.localization.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.rogue.blacksmith.util.kalman.KalmanFilter;

public class KalmanTwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    private final KalmanFilter
            headingFilter,
            wheelPos1Filter,
            wheelPos2Filter,
            wheelPos3Filter,
            headingVelocityFilter,
            wheelPos1VelocityFilter,
            wheelPos2VelocityFilter,
            wheelPos3VelocityFilter;
}
