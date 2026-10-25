package org.firstinspires.ftc.teamcode.simulated;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.sim.Pose2d;

/**
 * Measures, in ftc-sim, the numbers the Pedro "Foresight Tuner" would measure on a real robot:
 * top forward/strafe speed and how fast the robot coasts to a stop. Prints them in inches so they
 * can be compared with {@code Constants.foresightConfig}. Run with:
 * {@code ./gradlew -PftcSim :TeamCode:testSimulatedDebugUnitTest --tests '*ForesightSimMeasurementTest*'}
 */
class ForesightSimMeasurementTest {
    private static final double TICK = 0.01;
    private static final double IN = 0.0254;

    private void power(FakeHardwareMap hw, double f, double s) {
        // Pedro Mecanum mixing, s+ = left
        double[] p = {f - s, f + s, f + s, f - s};
        String[] n = {RobotConfig.FRONT_LEFT, RobotConfig.FRONT_RIGHT, RobotConfig.BACK_LEFT, RobotConfig.BACK_RIGHT};
        DcMotorSimple.Direction[] d = {RobotConfig.FRONT_LEFT_DIRECTION, RobotConfig.FRONT_RIGHT_DIRECTION,
                RobotConfig.BACK_LEFT_DIRECTION, RobotConfig.BACK_RIGHT_DIRECTION};
        for (int i = 0; i < 4; i++) {
            hw.motor(n[i]).setDirection(d[i]);
            hw.motor(n[i]).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hw.motor(n[i]).setPower(p[i]);
        }
    }

    private void measure(String label, double f, double s) {
        FakeHardwareMap hw = new BioBuzzSimRobot().create();
        hw.drive().setPose(new Pose2d(-0.6, -0.6, 0.0));
        power(hw, f, s);
        for (int i = 0; i < 60; i++) hw.advance(TICK);
        Pose2d a = hw.drive().pose();
        hw.advance(TICK);
        Pose2d b = hw.drive().pose();
        double v = Math.hypot(b.x() - a.x(), b.y() - a.y()) / TICK / IN;
        power(hw, 0, 0);
        Pose2d start = hw.drive().pose();
        double t = 0, prev = v;
        Pose2d last = start;
        while (t < 3) {
            hw.advance(TICK);
            t += TICK;
            Pose2d now = hw.drive().pose();
            double speed = Math.hypot(now.x() - last.x(), now.y() - last.y()) / TICK / IN;
            last = now;
            if (speed < 0.5) break;
            prev = speed;
        }
        double coast = Math.hypot(last.x() - start.x(), last.y() - start.y()) / IN;
        double decel = coast > 1e-6 ? v * v / (2 * coast) : Double.POSITIVE_INFINITY;
        System.out.printf("SIM-MEASURE %s: top speed %.1f in/s, coast to stop %.2f in in %.2f s, natural decel ~%.0f in/s^2%n",
                label, v, coast, t, decel);
    }

    @Test
    void measureForwardAndStrafe() {
        measure("forward", 1.0, 0.0);
        measure("strafe", 0.0, 1.0);
    }
}
