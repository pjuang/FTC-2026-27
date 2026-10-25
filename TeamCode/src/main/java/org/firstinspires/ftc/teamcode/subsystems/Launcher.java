package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Flywheel launcher plus an optional feeder that pushes POLLEN into the flywheel.
 *
 * <p>Missing hardware is a safe no-op: no flywheel means {@link #isReady()} is immediately true and
 * nothing spins; no feeder means feeding relies on the intake (which is how the ftc-sim robot
 * works: its intake pulls balls straight into the flywheel's mouth).</p>
 *
 * <p>BIOBUZZ rule reminder (Competition Manual TU06, G417): LAUNCHING into the upward-facing CELL is
 * the only allowed way to earn a HIVE TIP; do not launch into the bottom or side of a HIVE.</p>
 */
public class Launcher {
    private final DcMotorEx flywheel;
    private final DcMotorSimple feeder;
    private final ElapsedTime sinceSpinUp = new ElapsedTime();
    private double targetPower;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = OptionalHardware.get(hardwareMap, DcMotorEx.class, RobotConfig.LAUNCHER_NAME);
        if (flywheel != null) {
            flywheel.setDirection(RobotConfig.LAUNCHER_DIRECTION);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setPower(0);
        }
        feeder = OptionalHardware.get(hardwareMap, DcMotorSimple.class, RobotConfig.FEEDER_NAME);
        if (feeder != null) {
            feeder.setDirection(RobotConfig.FEEDER_DIRECTION);
            feeder.setPower(0);
        }
    }

    public boolean hasFlywheel() { return flywheel != null; }
    public boolean hasFeeder() { return feeder != null; }

    /** Start the flywheel at {@code power} (0..1). Calling again with the same power does not restart the timer. */
    public void spinUp(double power) {
        if (power != targetPower) sinceSpinUp.reset();
        targetPower = power;
        if (flywheel != null) flywheel.setPower(power);
    }

    public void stop() {
        targetPower = 0;
        if (flywheel != null) flywheel.setPower(0);
        feed(false);
    }

    /** Run the feeder (if there is one) to push balls into the flywheel. */
    public void feed(boolean on) {
        if (feeder != null) feeder.setPower(on ? RobotConfig.FEEDER_POWER : 0);
    }

    /** Measured flywheel speed in RPM, or 0 if there is no flywheel/encoder. */
    public double rpm() {
        if (flywheel == null) return 0;
        return flywheel.getVelocity() / RobotConfig.LAUNCHER_TICKS_PER_REV * 60.0;
    }

    /**
     * True once the flywheel has had {@link RobotConfig#LAUNCHER_SPINUP_S} seconds, or (if its
     * encoder is plugged in) once it reads within tolerance of the speed this power should give.
     */
    public boolean isReady() {
        if (flywheel == null) return true;
        if (targetPower <= 0) return false;
        if (sinceSpinUp.seconds() >= RobotConfig.LAUNCHER_SPINUP_S) return true;
        double expected = targetPower * RobotConfig.LAUNCHER_FREE_RPM;
        return sinceSpinUp.seconds() > 0.3
                && Math.abs(rpm()) >= expected * (1.0 - RobotConfig.LAUNCHER_READY_TOLERANCE);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", flywheel == null ? "MISSING (no-op): '" + RobotConfig.LAUNCHER_NAME + "'"
                : String.format("power %.2f  rpm %.0f  ready %b", targetPower, rpm(), isReady()));
        telemetry.addData("Feeder", feeder == null ? "none (intake feeds)" : "present");
    }
}
