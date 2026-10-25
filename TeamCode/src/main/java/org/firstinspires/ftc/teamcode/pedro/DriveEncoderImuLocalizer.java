package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Pedro Pathing v3 localizer that uses only the four mecanum drive-motor encoders plus the hub IMU.
 *
 * <p>Pedro v3 ships Pinpoint, OTOS, two-/three-wheel dead-wheel and OctoQuad localizers but no
 * drive-encoder one, so this class fills that gap. It is the default because it needs no extra
 * hardware and because ftc-sim simulates drive encoders and an IMU (it has no Pinpoint/OTOS).</p>
 *
 * <p>Accuracy: mecanum wheels slip, so expect a few inches of drift per long move. That is fine for
 * LEAVE/PARK and short shots; switch {@link RobotConfig#LOCALIZER} to PINPOINT or OTOS for more.</p>
 *
 * <p>Frames: Pedro's robot frame is +x forward, +y left, heading counter-clockwise in radians.
 * Units are inches (everything in this project uses inches).</p>
 */
public class DriveEncoderImuLocalizer implements Localizer {
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;
    private final double inchesPerTick;

    private Pose pose = Pose.zero();
    private Velocity velocity = Velocity.zero();
    private MotionState state = MotionState.zero();

    private int lastFl, lastFr, lastBl, lastBr;
    /** IMU yaw that corresponds to {@code headingOffset}; heading = offset + (yaw - yawAtOffset). */
    private double imuYawAtSetPose;
    private double headingAtSetPose;
    private double lastHeading;

    public DriveEncoderImuLocalizer(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_LEFT);
        fr = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_RIGHT);
        bl = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_LEFT);
        br = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_RIGHT);
        // Same directions as the drivetrain so a positive count means "this wheel rolled forward".
        fl.setDirection(RobotConfig.FRONT_LEFT_DIRECTION);
        fr.setDirection(RobotConfig.FRONT_RIGHT_DIRECTION);
        bl.setDirection(RobotConfig.BACK_LEFT_DIRECTION);
        br.setDirection(RobotConfig.BACK_RIGHT_DIRECTION);

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RobotConfig.HUB_LOGO, RobotConfig.HUB_USB)));

        inchesPerTick = Math.PI * RobotConfig.WHEEL_DIAMETER_IN * RobotConfig.DRIVE_GEAR_RATIO
                / RobotConfig.DRIVE_TICKS_PER_REV;
        reset();
    }

    private double rawYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private void latchEncoders() {
        lastFl = fl.getCurrentPosition();
        lastFr = fr.getCurrentPosition();
        lastBl = bl.getCurrentPosition();
        lastBr = br.getCurrentPosition();
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;
        headingAtSetPose = newPose.heading();
        imuYawAtSetPose = rawYaw();
        lastHeading = newPose.heading();
        latchEncoders();
        state = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void update() {
        int nFl = fl.getCurrentPosition(), nFr = fr.getCurrentPosition();
        int nBl = bl.getCurrentPosition(), nBr = br.getCurrentPosition();
        double dFl = (nFl - lastFl) * inchesPerTick, dFr = (nFr - lastFr) * inchesPerTick;
        double dBl = (nBl - lastBl) * inchesPerTick, dBr = (nBr - lastBr) * inchesPerTick;
        lastFl = nFl; lastFr = nFr; lastBl = nBl; lastBr = nBr;

        // Mecanum forward kinematics, matching Pedro's Mecanum mixing
        // (fl = f - s - t, fr = f + s + t, bl = f + s - t, br = f - s + t; s+ = left).
        double dForward = (dFl + dFr + dBl + dBr) / 4.0 * RobotConfig.FORWARD_MULTIPLIER;
        double dLeft = (-dFl + dFr + dBl - dBr) / 4.0 * RobotConfig.STRAFE_MULTIPLIER;

        // Heading straight from the IMU (no drift from wheel slip).
        double heading = normalize(headingAtSetPose + (rawYaw() - imuYawAtSetPose));
        double dHeading = normalize(heading - lastHeading);
        double mid = lastHeading + dHeading / 2.0;   // midpoint heading for the arc
        lastHeading = heading;

        double dx = dForward * Math.cos(mid) - dLeft * Math.sin(mid);
        double dy = dForward * Math.sin(mid) + dLeft * Math.cos(mid);
        pose = new Pose(pose.x() + dx, pose.y() + dy, heading);

        // Velocity from the motors' own velocity readings (ticks/s). These come from the hub (or
        // the simulator) rather than from this loop's timing, so a slow loop does not make the
        // velocity estimate noisy.
        double vFl = fl.getVelocity() * inchesPerTick, vFr = fr.getVelocity() * inchesPerTick;
        double vBl = bl.getVelocity() * inchesPerTick, vBr = br.getVelocity() * inchesPerTick;
        double vForward = (vFl + vFr + vBl + vBr) / 4.0 * RobotConfig.FORWARD_MULTIPLIER;
        double vLeft = (-vFl + vFr + vBl - vBr) / 4.0 * RobotConfig.STRAFE_MULTIPLIER;
        double omega = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        velocity = new Velocity(
                vForward * Math.cos(heading) - vLeft * Math.sin(heading),
                vForward * Math.sin(heading) + vLeft * Math.cos(heading),
                omega);
        state = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public void reset() {
        imu.resetYaw();
        velocity = Velocity.zero();
        setPose(Pose.zero());
    }

    private static double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
