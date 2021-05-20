package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS  = 1; // in
    public static double GEAR_RATIO    = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET   = -8.7; // in; offset of the lateral wheel
    public static double SIDE_OFFSET      = 1.5625;// 1.5+1/16 in; offset of left (or right) wheel
    public static int ADJ_ODOM_SIGN;



    public static final double X_MULTIPLIER = 0.979;
    public static final double Y_MULTIPLIER = 0.979;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, boolean qAdj) {
        super(Arrays.asList(
                new Pose2d(SIDE_OFFSET, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "s2"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "s1"));
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        //don't do that // leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        if(qAdj) {ADJ_ODOM_SIGN = -1;}
        else {ADJ_ODOM_SIGN = 1;}

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(ADJ_ODOM_SIGN*leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(ADJ_ODOM_SIGN*rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(ADJ_ODOM_SIGN*frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(ADJ_ODOM_SIGN*leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(ADJ_ODOM_SIGN*rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(ADJ_ODOM_SIGN*frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
