package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Every field position the autonomous uses, in ONE table, written for RED and mirrored for BLUE.
 *
 * <h2>Coordinate system (same as ftc-sim, so sim and robot agree)</h2>
 * <ul>
 *   <li>Units: inches. Origin: the centre of the field.</li>
 *   <li>+X points from the RED alliance wall toward the BLUE alliance wall.</li>
 *   <li>+Y points from the AUDIENCE wall toward the REAR wall.</li>
 *   <li>Heading: radians, counter-clockwise, 0 = facing +X (toward the blue wall).</li>
 * </ul>
 * <p>The BIOBUZZ field is rotationally symmetric (TU06 Figure 9-2): the blue version of a red pose
 * is the same pose rotated 180 degrees about the field centre, (x, y, h) -> (-x, -y, h + 180).</p>
 *
 * <p>Anything marked TODO-measure is an estimate from the manual figures / CAD and the simulator.
 * Measure it on a real field (tape measure from the walls) and fix it here; every Auto picks it
 * up.</p>
 */
public final class FieldPoses {
    private FieldPoses() {}

    // ---------- Field geometry ----------
    /** Inside face of the perimeter wall from the field centre. ftc-sim uses 3.5814 m / 2 = 70.5 in. TODO-measure. */
    public static double WALL = 70.5;
    /** HIVE centre line is 12.75 in from field centre toward its own alliance (ftc-sim BioBuzzHive, from CAD). TODO-measure. */
    public static double HIVE_X = -12.75;
    /** Half the robot's length / width, from RobotConfig. */
    public static double halfLength() { return RobotConfig.ROBOT_LENGTH_IN / 2.0; }
    public static double halfWidth() { return RobotConfig.ROBOT_WIDTH_IN / 2.0; }

    /** Clear of the HIVE frame (frame base reaches x = -24.8 in; robot half-width + margin). */
    public static double LANE_X = -42.0;

    private static final double FACE_REAR = Math.toRadians(90);      // facing +Y
    private static final double FACE_AUDIENCE = Math.toRadians(-90); // facing -Y
    private static final double FACE_BLUE_WALL = 0;                  // facing +X

    // ---------- RED poses ----------

    /**
     * NEAR start: back bumper touching the audience wall, centred behind our HIVE, facing it.
     * G304 A/B: own half (x &lt; 0) and touching the wall. Preloads (4 POLLEN) start inside the
     * robot or on the tiles touching it (TU06 10.3.1.A.iv / 10.3.4).
     */
    public static Pose redNearStart() {
        return new Pose(HIVE_X, -WALL + halfLength(), FACE_REAR);
    }

    /**
     * FAR start: back bumper touching our alliance wall at mid-field, facing the blue wall.
     * Clear of the LOADING ZONE (G304.D) and the alliance-wall FLOWER at y = -23 in (G304.C).
     */
    public static Pose redFarStart() {
        return new Pose(-WALL + halfLength(), 0.0, FACE_BLUE_WALL);
    }

    /**
     * Where the robot centre sits when it LAUNCHES at the audience-side CELL of our HIVE.
     * At match start the red HIVE's AUDIENCE CELL is the upward one (TU06 10.3.1). From here
     * a RobotConfig.LAUNCHER_POWER shot lands in it in ftc-sim. TODO-measure + tune on the real field.
     */
    public static Pose redShootAudience() {
        return new Pose(HIVE_X, -58.5, FACE_REAR);
    }

    /**
     * Shooting spot on the REAR side, for the second volley, aimed at the SCORING CELL which is the
     * upward one only AFTER our HIVE has tipped (G417: never launch into the down CELL).
     * x is 2.9 in toward the centre of HIVE_X to stay clear of the rear-wall FLOWER at
     * (-23.2, 68.1). TODO-measure.
     */
    public static Pose redShootRear() {
        return new Pose(HIVE_X + 2.9, 58.5, FACE_AUDIENCE);
    }

    /**
     * Distance to back away from the shooting spot before spinning up, then creep back in.
     * Needed in ftc-sim, where a ball already in the flywheel's mouth is thrown at whatever speed
     * the wheel has (so a ball in the mouth during spin-up falls short). A real robot whose
     * POLLEN wait in a hopper behind a feeder can set this to 0.
     */
    public static double SHOT_BACKOFF_IN = 3.5;

    /**
     * Red GARDEN: 4 POLLEN in a line against the AUDIENCE wall starting in the corner nearest our
     * alliance area (TU06 10.3.1.A.ii and the note after it). This is where to stand, facing the
     * wall, to sweep all four. TODO-measure.
     */
    public static Pose redGardenApproach() {
        return new Pose(-WALL + halfWidth() + 0.3, -48.0, FACE_AUDIENCE);
    }

    public static Pose redGardenPickup() {
        return new Pose(-WALL + halfWidth() + 0.3, -59.5, FACE_AUDIENCE);
    }

    /** Waypoints from the GARDEN to the rear shooting spot, outside the HIVE frame. */
    public static Pose redLaneAudience() { return new Pose(LANE_X, -20.0, FACE_AUDIENCE); }
    public static Pose redLaneRear() { return new Pose(LANE_X, 30.0, FACE_AUDIENCE); }

    /**
     * PARK: partially inside our LOADING ZONE and NOT touching the wall, which scores both
     * LEAVE (3) and AUTO PARK (5) (TU06 10.5.4, Table 10-2).
     * LOADING ZONE: ~23 in along the alliance wall by 11 in deep (TU06 9.3); from Figure 9-2 it
     * runs from about y = +23.6 to +46.5 in on the red wall. TODO-measure (tape lines).
     * Robot edge is about 5 in inside the zone line and 5.6 in off the wall.
     */
    public static Pose redPark() {
        return new Pose(-WALL + 11.0 + halfWidth() - 5.0 - 0.5, 35.0, FACE_AUDIENCE);
    }

    // ---------- Mirroring ----------

    /** Red pose for RED; the same spot rotated 180 degrees about the centre for BLUE. */
    public static Pose forAlliance(Alliance alliance, Pose red) {
        if (alliance == Alliance.RED) return red;
        return new Pose(-red.x(), -red.y(), normalize(red.heading() + Math.PI));
    }

    public static Pose start(Alliance a, StartPosition s) {
        return forAlliance(a, s == StartPosition.NEAR ? redNearStart() : redFarStart());
    }

    static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
