

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
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192 / 4;
    public static double WHEEL_RADIUS = 2.3622204 / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double LATERAL_FORWARD_ENCODER_D = 0.27;//0.8;
    public static double LATERAL_FORWARD_LEFT = 0.5;//0.74;
    public static double LATERAL_FORWARD_RIGHT = -1.2;//-1.41;
    public static double LATERAL_DISTANCE = 14;//13.8;//9.4; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4;//3.7; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1    ; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public ThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(LATERAL_FORWARD_LEFT, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(LATERAL_FORWARD_RIGHT, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, LATERAL_FORWARD_ENCODER_D, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backright"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontright"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontleft"));
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
