package org.firstinspires.ftc.teamcode.auto;

/**
 * Where on our half the robot starts.
 *
 * <p>The BIOBUZZ manual (TU06, G304) does NOT define fixed starting tiles: the robot must start on
 * its own half of the field (columns A-C red, D-F blue) touching a perimeter wall, not touching a
 * FLOWER and not in the LOADING ZONE. These two are just the two spots this code supports.</p>
 */
public enum StartPosition {
    /** Against the audience wall, lined up behind our HIVE. Shortest path to the first shot. */
    NEAR,
    /** Against our alliance wall, middle of the field. Use if a partner wants NEAR. */
    FAR
}
