//package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.opmode;
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.roadrunner.localization.*;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.OpModes.Autonomous.RoadRunner.drive.TwoWheelTrackingLocalizer;
//
//import java.util.Arrays;
//import java.util.List;
//
//import ftc.rogue.blacksmith.util.kalman.KalmanFilter;
//
//public class KalmanTwoWheelTrackingLocalizer extends TwoWheelTrackingLocalizer {
//    private final KalmanFilter  headingFilter, headingVelocityFilter, wheelPos1Filter, wheelPos2Filter,wheelPos1VelocityFilter, wheelPos2VelocityFilter;
//    private List<Pose2d> filteredPose;
//    public static double pos1VelocityFilterR, pos1VelocityFilterQ, pos2VelocityFilterR, pos2VelocityFilterQ, pos1FilterR, pos1FilterQ, pos2FilterR, pos2FilterQ;
//
//    public KalmanTwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive)
//    {
//        super(hardwareMap, drive);
//
//        // The KalmanFilter R and Q values refer to how you rely on either your sensor output or prediction output.
//
//        // The process noise (R) refers to how much noise is in the actual system.
//        //This characterizes the system, it's what's actually happening in the real world.
//        // This can be caused by a ever-so-slight jitter in the motor or fluctuations in battery voltage,
//        // or any number of things actually.
//
//        // The measurement noise (Q) is how much noise is in the system output. This is caused due to slight
//        // inaccuracies that are inherent in all sensors/systems, and can become pretty significant over time.
//
//        // Lastly, remember that the heading is in radians.
//
//        headingVelocityFilter = new KalmanFilter(0,0);
//        headingFilter = new KalmanFilter(0,0);
//        wheelPos1Filter = new KalmanFilter(0,0);
//        wheelPos2Filter = new KalmanFilter(0,0);
//        wheelPos1VelocityFilter = new KalmanFilter(0,0);
//        wheelPos2VelocityFilter = new KalmanFilter(0,0);
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getFilterWheelPositions() {
//        super.getWheelPositions();
//        wheelPos1Filter.filter()
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelVelocities() {
//        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
//        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
//        //  compensation method
//
//        return Arrays.asList(
//                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
//        );
//    }
//}
