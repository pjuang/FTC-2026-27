package org.firstinspires.ftc.teamcode.simulated;

import static org.junit.jupiter.api.Assertions.assertNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.auto.BlueAuto;
import org.firstinspires.ftc.teamcode.auto.BlueFarAuto;
import org.firstinspires.ftc.teamcode.auto.FieldPoses;
import org.firstinspires.ftc.teamcode.auto.LeaveOnlyAuto;
import org.firstinspires.ftc.teamcode.auto.RedAuto;
import org.firstinspires.ftc.teamcode.auto.RedFarAuto;
import org.firstinspires.ftc.teamcode.auto.StartPosition;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.physics.PivotState;
import org.ngicollective.ftcsim.season.BioBuzzField;
import org.ngicollective.ftcsim.season.BioBuzzScenario;
import org.ngicollective.ftcsim.season.BioBuzzScore;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Locale;

/**
 * Runs each BIOBUZZ Auto headless in ftc-sim for a full 30 s AUTO period and reports what the sim
 * scored, where the robot ended, and the LEAVE / PARK points computed from that end pose (ftc-sim
 * does not score LEAVE or PARK itself).
 *
 * <p><b>Real-time pacing.</b> Pedro's Follower and the SDK's ElapsedTime both read
 * {@code System.nanoTime()}, which the harness does not virtualise. So each 20 ms sim tick waits
 * until 20 ms of wall-clock time has also passed; one run takes ~33 s.</p>
 *
 * <p>Run: {@code bash ./gradlew -PftcSim :TeamCode:testSimulatedDebugUnitTest --tests '*BioBuzzAutoSimTest*'}.
 * Results: {@code TeamCode/build/sim-results/*.txt} and {@code *.csv} (pose trace, metres).</p>
 */
class BioBuzzAutoSimTest {
    private static final double TICK = 0.02;
    private static final double IN = 0.0254;

    @Test void redNear()   { run("RedAuto", new RedAuto(), "biobuzz-auto-red-near", Alliance.RED, StartPosition.NEAR); }
    @Test void blueNear()  { run("BlueAuto", new BlueAuto(), "biobuzz-auto-blue-near", Alliance.BLUE, StartPosition.NEAR); }
    @Test void redFar()    { run("RedFarAuto", new RedFarAuto(), "biobuzz-auto-red-far", Alliance.RED, StartPosition.FAR); }
    @Test void blueFar()   { run("BlueFarAuto", new BlueFarAuto(), "biobuzz-auto-blue-far", Alliance.BLUE, StartPosition.FAR); }
    @Test void leaveOnly() { run("LeaveOnlyAuto", new LeaveOnlyAuto(), "biobuzz-auto-red-near", Alliance.RED, StartPosition.NEAR); }

    private void run(String label, LinearOpMode opMode, String scenarioName, Alliance alliance, StartPosition start) {
        BioBuzzScenario scenario = BioBuzzScenario.named(scenarioName);
        BioBuzzSimRobot robotDef = new BioBuzzSimRobot(SimConfigFiles.robot(BioBuzzSimRobot.CONFIG_NAME),
                SimConfigFiles.field(), scenario.scene());
        FakeHardwareMap hw = robotDef.create();
        SimulatedScene scene = scenario.scene();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                SimConfigFiles.field(), robotDef.config());
        hw.setPhysics(world);

        Pose s = FieldPoses.start(alliance, start);
        hw.drive().setPose(new Pose2d(s.x() * IN, s.y() * IN, s.heading()));

        StringBuilder trace = new StringBuilder("t,x_m,y_m,heading_rad,est_x_m,est_y_m,est_heading_rad\n");
        LinearOpModeHarness harness = OpModeHarness.forLinear(opMode, hw);
        harness.launch();
        pace(harness, hw, 1.0, null, 0, opMode);          // INIT for 1 s
        BioBuzzScore baseline = score(scenario, world);   // pre-staged NECTAR already in CELLs
        harness.pressStart();
        long t0 = System.nanoTime();
        pace(harness, hw, 30.0, trace, t0, opMode);       // the 30 s AUTO period (manual 10.1)
        Pose2d endPose = hw.drive().pose();
        Pose endEstimate = estimate(opMode);
        BioBuzzScore at30 = score(scenario, world);
        harness.pressStop();
        // Let balls already in the air land (refs assess when the field is at rest).
        for (int i = 0; i < 150; i++) { hw.advance(TICK); }
        BioBuzzScore settled = score(scenario, world);
        harness.awaitCompletion(2000);

        boolean red = alliance == Alliance.RED;
        boolean leave = !touchesWall(endPose);
        boolean park = inLoadingZone(endPose, red);
        int simPoints = red ? settled.redPoints() : settled.bluePoints();
        int earned = simPoints - (red ? baseline.redPoints() : baseline.bluePoints());
        int total = earned + (leave ? 3 : 0) + (park ? 5 : 0);
        String report = String.format(Locale.ROOT,
                "%s on %s (%s):%n  end pose: x=%.1f in, y=%.1f in, heading=%.0f deg (Pedro estimate: %s)%n"
                        + "  sim score before START (pre-staged): red %d / blue %d%n"
                        + "  sim score at t=30s: red %d / blue %d; after settling: red %d / blue %d%n"
                        + "  sim breakdown: %s%n"
                        + "  computed from end pose: LEAVE=%s (+3), AUTO PARK=%s (+5)%n"
                        + "  => %s AUTO points earned by the robot: %d (sim delta %d + LEAVE/PARK)%n  OpMode failure: %s%n",
                label, scenarioName, alliance,
                endPose.x() / IN, endPose.y() / IN, Math.toDegrees(endPose.heading()),
                endEstimate == null ? "n/a" : String.format(Locale.ROOT, "%.1f, %.1f, %.0f deg",
                        endEstimate.x(), endEstimate.y(), Math.toDegrees(endEstimate.heading())),
                baseline.redPoints(), baseline.bluePoints(),
                at30.redPoints(), at30.bluePoints(), settled.redPoints(), settled.bluePoints(),
                settled, leave, park, alliance, total, earned, harness.failure());
        System.out.println("SIM-RESULT " + report);
        write(label + ".txt", report);
        write(label + ".csv", trace.toString());
        assertNull(harness.failure(), "the OpMode threw");
    }

    private static BioBuzzScore score(BioBuzzScenario scenario, FieldPhysics world) {
        return BioBuzzScore.of(
                BioBuzzField.scene(scenario.redHive(), scenario.blueHive())
                        .tipped(PivotState.anglesOf(world.pivots())).scoringVolumes(),
                world.elements(), PivotState.swingsOf(world.pivots()));
    }

    /** Real-time paced stepping: sim time never runs ahead of the wall clock. */
    private static void pace(LinearOpModeHarness h, FakeHardwareMap hw, double seconds, StringBuilder trace, long t0, Object opMode) {
        long start = System.nanoTime();
        int ticks = (int) Math.round(seconds / TICK);
        for (int i = 1; i <= ticks; i++) {
            h.advance(TICK);
            h.eventLoopIteration();
            long due = start + (long) (i * TICK * 1e9);
            long wait = due - System.nanoTime();
            if (wait > 0) {
                try { Thread.sleep(wait / 1_000_000L, (int) (wait % 1_000_000L)); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
            }
            if (trace != null && i % 5 == 0) {
                Pose2d p = hw.drive().pose();
                Pose e = estimate(opMode);
                trace.append(String.format(Locale.ROOT, "%.2f,%.4f,%.4f,%.4f,%s%n",
                        (System.nanoTime() - t0) / 1e9, p.x(), p.y(), p.heading(),
                        e == null ? ",," : String.format(Locale.ROOT, "%.4f,%.4f,%.4f", e.x() * IN, e.y() * IN, e.heading())));
            }
        }
    }

    /** Pedro's own pose estimate, read from the Auto's protected {@code drive} field (null for LeaveOnlyAuto). */
    private static Pose estimate(Object opMode) {
        try {
            java.lang.reflect.Field f = org.firstinspires.ftc.teamcode.auto.BioBuzzAuto.class.getDeclaredField("drive");
            if (!(opMode instanceof org.firstinspires.ftc.teamcode.auto.BioBuzzAuto)) return null;
            f.setAccessible(true);
            org.firstinspires.ftc.teamcode.subsystems.Drive d = (org.firstinspires.ftc.teamcode.subsystems.Drive) f.get(opMode);
            return d == null ? null : d.pose();
        } catch (ReflectiveOperationException | RuntimeException e) {
            return null;
        }
    }

    /** Any corner of the robot footprint within 0.5 in of a perimeter wall counts as contact. */
    private static boolean touchesWall(Pose2d p) {
        double wall = FieldPoses.WALL * IN;
        for (double[] c : corners(p)) {
            if (Math.abs(c[0]) > wall - 0.5 * IN || Math.abs(c[1]) > wall - 0.5 * IN) return true;
        }
        return false;
    }

    /**
     * LOADING ZONE (manual 9.3, Fig 10-7): ~11 in deep along the alliance-side wall. Red is
     * x in [-70.5, -59.5] in, y in [+23.6, +46.5] in; blue is the point mirror. TODO-measure
     * against the field drawings. "At least partially in": any corner or the centre inside.
     */
    private static boolean inLoadingZone(Pose2d p, boolean red) {
        double s = red ? 1 : -1;
        double xa = s * -70.5 * IN, xb = s * -59.5 * IN, ya = s * 23.6 * IN, yb = s * 46.5 * IN;
        double x0 = Math.min(xa, xb), x1 = Math.max(xa, xb), y0 = Math.min(ya, yb), y1 = Math.max(ya, yb);
        double[][] cs = corners(p);
        double[][] pts = {cs[0], cs[1], cs[2], cs[3], {p.x(), p.y()}};
        for (double[] c : pts) {
            if (c[0] >= x0 && c[0] <= x1 && c[1] >= y0 && c[1] <= y1) return true;
        }
        return false;
    }

    private static double[][] corners(Pose2d p) {
        double hl = org.firstinspires.ftc.teamcode.config.RobotConfig.ROBOT_LENGTH_IN / 2 * IN;
        double hw = org.firstinspires.ftc.teamcode.config.RobotConfig.ROBOT_WIDTH_IN / 2 * IN;
        double c = Math.cos(p.heading()), s = Math.sin(p.heading());
        double[][] out = new double[4][];
        int k = 0;
        for (double a : new double[] {-hl, hl}) for (double b : new double[] {-hw, hw}) {
            out[k++] = new double[] {p.x() + a * c - b * s, p.y() + a * s + b * c};
        }
        return out;
    }

    private static void write(String name, String body) {
        try {
            Path dir = Paths.get("build", "sim-results");
            Files.createDirectories(dir);
            Files.write(dir.resolve(name), body.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
