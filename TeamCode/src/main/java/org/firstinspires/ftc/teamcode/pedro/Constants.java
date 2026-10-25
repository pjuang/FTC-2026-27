package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Pedro Pathing v3 setup, in the layout of the official Pedro Quickstart
 * (github.com/Pedro-Pathing/Quickstart, {@code pedro/Constants.java}): one place that builds the
 * drivetrain, the localizer and the Foresight path-following algorithm, and a
 * {@link #create(HardwareMap)} that every OpMode calls to get a {@link Follower}.
 *
 * <p>Pedro v3 note: older guides (and RobToT) use {@code FollowerConstants}/{@code FollowerBuilder}
 * from Pedro 1.x/2.x. v3 replaced them with {@code new Follower(localizer, drivetrain,
 * new Foresight(config))}; do not mix code from the two.</p>
 *
 * <p>All distances are INCHES, velocities in/s, decelerations in/s^2.</p>
 */
public class Constants {

    // ------------------------------------------------------------------------------------------
    // Drivetrain: names and directions come from RobotConfig (same as BasicTeleop).
    // The "Mecanum Tuner" procedure checks directions on the real robot.
    // ------------------------------------------------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.FRONT_LEFT);
        c.frontRightName.set(RobotConfig.FRONT_RIGHT);
        c.backLeftName.set(RobotConfig.BACK_LEFT);
        c.backRightName.set(RobotConfig.BACK_RIGHT);
        c.frontLeftDirection.set(RobotConfig.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(RobotConfig.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(RobotConfig.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(RobotConfig.BACK_RIGHT_DIRECTION);
    });

    // ------------------------------------------------------------------------------------------
    // Optional localizers. Only built when RobotConfig.LOCALIZER selects them, so a robot without
    // a Pinpoint/OTOS never touches these devices. Fill the values with the matching tuner.
    // ------------------------------------------------------------------------------------------
    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.PINPOINT_NAME);
        c.xPodOffset.set(RobotConfig.PINPOINT_X_POD_OFFSET_IN);
        c.yPodOffset.set(RobotConfig.PINPOINT_Y_POD_OFFSET_IN);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.xPodDirection.set(RobotConfig.PINPOINT_X_POD_REVERSED
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(RobotConfig.PINPOINT_Y_POD_REVERSED
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); // TODO-measure pod type
    });

    public static OTOSConfig otosConfig = new OTOSConfig(c -> {
        c.name.set(RobotConfig.OTOS_NAME);
        c.linearUnit.set(DistanceUnit.INCH);
        c.offset.set(new Pose(RobotConfig.OTOS_OFFSET_X_IN, RobotConfig.OTOS_OFFSET_Y_IN,
                Math.toRadians(RobotConfig.OTOS_OFFSET_HEADING_DEG)));
        c.linearScalar.set(RobotConfig.OTOS_LINEAR_SCALAR);
        c.angularScalar.set(RobotConfig.OTOS_ANGULAR_SCALAR);
    });

    // ------------------------------------------------------------------------------------------
    // Foresight (the path follower). TODO-tune on the real robot with the "Foresight Tuner"
    // procedure, then paste the code it generates over this block.
    //
    // maxAchievable*Velocity and natural*Deceleration were MEASURED IN ftc-sim on the simulated
    // 312 rpm / 96 mm mecanum robot (TeamCode/robot-config/biobuzz.json) by
    // ForesightSimMeasurementTest (src/testSimulated). The controller gains are starting guesses
    // that were only checked by running the BIOBUZZ Autos in the sim. A real robot with the same
    // motors and wheels is usually in the same ballpark, but NONE of this is tuned for any robot.
    // ------------------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.12);
        Controller secondaryTranslationalForward = Controller.proportional(0.25);
        Controller primaryTranslationalLateral = Controller.proportional(0.12);
        Controller secondaryTranslationalLateral = Controller.proportional(0.25);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.2));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.0));

        c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002, 0.002));

        c.maxAchievableForwardVelocity.set(49.0);
        c.maxAchievableStrafeVelocity.set(39.0);
        c.naturalForwardDeceleration.set(50.0);
        c.naturalStrafeDeceleration.set(41.0);
    });

    // ------------------------------------------------------------------------------------------
    // Factories (also used by Tuning.java).
    // ------------------------------------------------------------------------------------------
    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        switch (RobotConfig.LOCALIZER) {
            case PINPOINT:
                return new PinpointLocalizer(hardwareMap, pinpointConfig);
            case OTOS:
                return new OTOSLocalizer(hardwareMap, otosConfig);
            case DRIVE_ENCODER_IMU:
            default:
                return new DriveEncoderImuLocalizer(hardwareMap);
        }
    }

    public static Foresight algorithm() {
        return new Foresight(foresightConfig);
    }

    /** The one call an OpMode makes: {@code Follower follower = Constants.create(hardwareMap);} */
    public static Follower create(HardwareMap hardwareMap) {
        Drivetrain drivetrain = drivetrain(hardwareMap);
        Localizer localizer = localizer(hardwareMap);
        return new Follower(localizer, drivetrain, algorithm());
    }
}
