package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import java.util.ArrayList;
import java.util.List;

/**
 * The BIOBUZZ autonomous, shared by RedAuto, BlueAuto, RedFarAuto and BlueFarAuto.
 *
 * <h2>Plan (Competition Manual TU06)</h2>
 * <ol>
 *   <li><b>Preloads -&gt; HIVE.</b> Launch the 4 preloaded POLLEN (10.3.4) into our HIVE's upward
 *       AUDIENCE CELL. Enough weight tips the HIVE: a HIVE TIP completed before TELEOP counts as
 *       AUTO (10.5 B) and is worth 20 (Table 10-2). Only LAUNCHING into the upward CELL may cause a
 *       tip (G417).</li>
 *   <li><b>Optional GARDEN cycle</b> (only if enough time is left): sweep the 4 POLLEN in our
 *       GARDEN and launch them from the rear side into the SCORING CELL, which is the upward one
 *       once the HIVE has tipped. ASSUMES THE PRELOADS TIPPED THE HIVE (set DO_GARDEN_CYCLE false
 *       otherwise, or G417 says we would be launching into a down CELL).</li>
 *   <li><b>Park.</b> End AUTO partly inside our LOADING ZONE and off the wall: LEAVE 3 + PARK 5,
 *       assessed at the end of AUTO (10.5 F, 10.5.4).</li>
 * </ol>
 *
 * <h2>Timer guard</h2>
 * <p>AUTO is 30 s (10.1). The moment fewer than {@link RobotConfig#PARK_RESERVE_S} seconds are
 * left, whatever is running is abandoned and the robot drives to park. So LEAVE/PARK is attempted
 * even if a shot jams. The robot stops before 30 s and stays motionless (G403).</p>
 *
 * <p>No NECTAR goes into FLOWERs in AUTO (G410), we never cross to the blue half (G402), and we
 * control at most 4 elements at a time (G407): the plan only ever carries our own 4.</p>
 */
public abstract class BioBuzzAuto extends LinearOpMode {

    /** Set false if the preload volley does NOT tip the HIVE on the real field (see class doc). */
    public static boolean DO_GARDEN_CYCLE = true;

    protected abstract Alliance alliance();

    protected abstract StartPosition startPosition();

    protected Drive drive;
    protected Intake intake;
    protected Launcher launcher;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final List<Step> plan = new ArrayList<>();
    private int stepIndex = -1;
    private Step current;
    private boolean parking;
    private String status = "init";

    // ------------------------------------------------------------------------------------------
    // Poses for THIS alliance (red table mirrored for blue).
    // ------------------------------------------------------------------------------------------
    protected Pose p(Pose red) { return FieldPoses.forAlliance(alliance(), red); }

    protected double timeLeft() { return RobotConfig.AUTO_LENGTH_S - matchTimer.seconds(); }

    // ------------------------------------------------------------------------------------------
    // The plan. Override in a subclass to change the strategy.
    // ------------------------------------------------------------------------------------------
    protected void buildPlan(List<Step> steps) {
        if (startPosition() == StartPosition.FAR) {
            // Carry the preloads (intake on keeps tile-preloads against the robot) around the
            // HIVE frame to the audience side.
            steps.add(new DriveStep("far -> audience lane", RobotConfig.CARRY_SPEED, true,
                    p(new Pose(FieldPoses.LANE_X, -40.0, Math.toRadians(90)))));
        }
        steps.add(new ShootStep("preloads -> AUDIENCE CELL", p(FieldPoses.redShootAudience())));

        if (DO_GARDEN_CYCLE) {
            Step approach = new DriveStep("-> GARDEN", 1.0, false, p(FieldPoses.redGardenApproach()));
            approach.minTimeLeftToStart = RobotConfig.CYCLE_MIN_TIME_S;   // optional: skip if late
            steps.add(approach);
            steps.add(new DriveStep("sweep GARDEN", RobotConfig.CREEP_SPEED, true,
                    p(FieldPoses.redGardenPickup())));
            steps.add(new DriveStep("GARDEN -> rear", RobotConfig.CARRY_SPEED, true,
                    p(FieldPoses.redLaneAudience()), p(FieldPoses.redLaneRear())));
            steps.add(new ShootStep("GARDEN -> SCORING CELL", p(FieldPoses.redShootRear())));
        }
    }

    /** Drive to the LOADING ZONE from wherever we are, around the HIVE frame. */
    protected Step parkStep() {
        Pose here = drive.pose();
        Pose park = p(FieldPoses.redPark());
        // Work in red coordinates to pick a route, then mirror.
        Pose redHere = FieldPoses.forAlliance(alliance(), here); // mirroring twice = identity
        List<Pose> route = new ArrayList<>();
        if (redHere.x() > FieldPoses.LANE_X + 4) {
            // Near the HIVE: first step off the audience/rear wall (the rear-wall FLOWERs sit at
            // y ~ +/-68 in, and brushing one makes the wheels slip and the pose estimate jump),
            // then slide out to the lane (pure -X move, stays clear of the frame).
            double y = Math.max(-50.0, Math.min(50.0, redHere.y()));
            if (y != redHere.y()) {
                route.add(p(new Pose(redHere.x(), y, FieldPoses.redPark().heading())));
            }
            route.add(p(new Pose(FieldPoses.LANE_X, y, FieldPoses.redPark().heading())));
        }
        route.add(park);
        return new DriveStep("PARK", 1.0, false, route.toArray(new Pose[0]));
    }

    // ------------------------------------------------------------------------------------------
    // OpMode
    // ------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        Pose start = FieldPoses.start(alliance(), startPosition());
        drive.setPose(start);

        while (opModeInInit()) {           // G304.H: robot motionless after INIT
            drive.update();
            telemetry.addLine("BIOBUZZ " + alliance() + " " + startPosition());
            telemetry.addData("Place robot at (in)", String.format("x %.1f y %.1f h %.0f deg",
                    start.x(), start.y(), Math.toDegrees(start.heading())));
            intake.addTelemetry(telemetry);
            launcher.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) return;

        matchTimer.reset();
        drive.setPose(start);
        buildPlan(plan);
        nextStep();

        while (opModeIsActive()) {
            drive.update();

            if (!parking && timeLeft() < RobotConfig.PARK_RESERVE_S) {
                startPark("timer guard");
            }
            if (current != null && current.update()) {
                if (parking) {
                    current = null;
                    status = "parked";
                    intake.stop();
                    launcher.stop();
                } else {
                    nextStep();
                }
            }
            if (timeLeft() <= 0.3 && !"stopped".equals(status)) {
                // Stand still for the AUTO->TELEOP transition (G403).
                drive.stop();
                intake.stop();
                launcher.stop();
                current = null;
                status = "stopped";
            }

            telemetry.addData("Time left", String.format("%.1f s", timeLeft()));
            telemetry.addData("Step", current == null ? status : current.name);
            drive.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            launcher.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop();
        intake.stop();
        launcher.stop();
    }

    private void nextStep() {
        while (true) {
            stepIndex++;
            if (stepIndex >= plan.size()) {
                startPark("plan done");
                return;
            }
            Step s = plan.get(stepIndex);
            if (timeLeft() < s.minTimeLeftToStart + RobotConfig.PARK_RESERVE_S) {
                startPark("skipped '" + s.name + "' (not enough time)");
                return;
            }
            current = s;
            current.start();
            return;
        }
    }

    private void startPark(String why) {
        parking = true;
        status = why;
        launcher.stop();
        intake.stop();
        current = parkStep();
        current.start();
    }

    // ------------------------------------------------------------------------------------------
    // Steps
    // ------------------------------------------------------------------------------------------
    /** One thing the robot does. {@link #update()} returns true when finished. */
    protected abstract static class Step {
        final String name;
        /** Skip this step (and go park) unless at least this many seconds remain beyond the park reserve. */
        double minTimeLeftToStart = 0;
        final ElapsedTime timer = new ElapsedTime();

        Step(String name) { this.name = name; }

        void start() { timer.reset(); onStart(); }

        abstract void onStart();

        abstract boolean update();
    }

    /** Follow straight lines through waypoints; optionally run the intake while driving. */
    protected class DriveStep extends Step {
        private final double speed;
        private final boolean intakeOn;
        private final Pose[] waypoints;
        /** Give up (and move on) if the path has not finished in this many seconds. */
        double timeoutS = 8.0;

        DriveStep(String name, double speed, boolean intakeOn, Pose... waypoints) {
            super(name);
            this.speed = speed;
            this.intakeOn = intakeOn;
            this.waypoints = waypoints;
        }

        @Override
        void onStart() {
            if (intakeOn) intake.in(); else intake.stop();
            drive.driveThrough(speed, waypoints);
        }

        @Override
        boolean update() {
            return !drive.isBusy() || timer.seconds() > timeoutS;
        }
    }

    /**
     * Launch whatever we carry from {@code shootPose}:
     * drive there (intake on, keeping POLLEN against the robot) -&gt; back off
     * {@link FieldPoses#SHOT_BACKOFF_IN} and spin up -&gt; creep back in while feeding -&gt; keep
     * feeding for {@link RobotConfig#FIRE_DURATION_S} -&gt; stop the flywheel.
     */
    protected class ShootStep extends Step {
        private final Pose shootPose;
        private int phase;
        private final ElapsedTime phaseTimer = new ElapsedTime();

        ShootStep(String name, Pose shootPose) {
            super(name);
            this.shootPose = shootPose;
        }

        private Pose readyPose() {
            double back = FieldPoses.SHOT_BACKOFF_IN;
            return new Pose(shootPose.x() - back * Math.cos(shootPose.heading()),
                    shootPose.y() - back * Math.sin(shootPose.heading()), shootPose.heading());
        }

        @Override
        void onStart() {
            phase = 0;
            intake.in();
            drive.driveThrough(1.0, shootPose);
            if (FieldPoses.SHOT_BACKOFF_IN <= 0) launcher.spinUp(RobotConfig.LAUNCHER_POWER);
            phaseTimer.reset();
        }

        private void go(int next) { phase = next; phaseTimer.reset(); }

        @Override
        boolean update() {
            switch (phase) {
                case 0: // arrive at the shooting spot
                    if (!drive.isBusy() || phaseTimer.seconds() > 6) {
                        if (FieldPoses.SHOT_BACKOFF_IN > 0) {
                            intake.stop();
                            drive.driveThrough(RobotConfig.CREEP_SPEED, readyPose());
                            go(1);
                        } else {
                            go(2);
                        }
                    }
                    return false;
                case 1: // backing off; spin up once clear of the balls
                    if (!drive.isBusy() || phaseTimer.seconds() > 2) {
                        launcher.spinUp(RobotConfig.LAUNCHER_POWER);
                        go(2);
                    }
                    return false;
                case 2: // wait for the flywheel
                    if (launcher.isReady() || phaseTimer.seconds() > RobotConfig.LAUNCHER_SPINUP_S + 1.0) {
                        intake.in();
                        launcher.feed(true);
                        if (FieldPoses.SHOT_BACKOFF_IN > 0) {
                            drive.driveThrough(RobotConfig.CREEP_SPEED, shootPose);
                        }
                        go(3);
                    }
                    return false;
                case 3: // feeding / creeping in
                    if (phaseTimer.seconds() > RobotConfig.FIRE_DURATION_S && !drive.isBusy()) {
                        launcher.stop();
                        intake.stop();
                        return true;
                    }
                    return false;
                default:
                    return true;
            }
        }
    }
}
