package org.firstinspires.ftc.teamcode.simulated;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.sim.Pose2d;

import java.util.Locale;

/** Prints the simulator's true pose next to Pedro's estimate while following short paths. */
class FollowerSimDebugTest {
    private static final double IN = 0.0254;

    private void follow(String label, Pose start, Pose... to) throws InterruptedException {
        FakeHardwareMap hw = new BioBuzzSimRobot().create();
        hw.drive().setPose(new Pose2d(start.x() * IN, start.y() * IN, start.heading()));
        Drive drive = new Drive(hw);
        drive.setPose(start);
        drive.driveThrough(1.0, to);
        long t0 = System.nanoTime();
        for (int i = 1; i <= 100; i++) {
            hw.advance(0.02);
            drive.update();
            long wait = t0 + i * 20_000_000L - System.nanoTime();
            if (wait > 0) Thread.sleep(wait / 1_000_000L);
            if (i % 10 == 0) {
                Pose2d t = hw.drive().pose();
                Pose e = drive.pose();
                System.out.printf(Locale.ROOT, "DBG %s t=%.1f true(%.1f,%.1f,%.0f) est(%.1f,%.1f,%.0f) busy=%s%n",
                        label, i * 0.02, t.x() / IN, t.y() / IN, Math.toDegrees(t.heading()),
                        e.x(), e.y(), Math.toDegrees(e.heading()), drive.isBusy());
                StringBuilder m = new StringBuilder("DBG   motors");
                for (String n : new String[] {"fLeft", "fRight", "bLeft", "bRight"}) {
                    com.qualcomm.robotcore.hardware.DcMotorEx mo = hw.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, n);
                    m.append(String.format(Locale.ROOT, " %s p=%.3f pos=%d v=%.0f", n, mo.getPower(), mo.getCurrentPosition(), mo.getVelocity()));
                }
                System.out.println(m);
            }
        }
    }

    @Test
    void lines() throws InterruptedException {
        follow("fwd24", new Pose(0, -40, Math.PI / 2), new Pose(0, -16, Math.PI / 2));
        follow("turn", new Pose(-30, -30, Math.PI / 2), new Pose(-30, -10, -Math.PI / 2));
    }
}
