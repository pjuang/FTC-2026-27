package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * EVERY robot assumption in one place.
 *
 * <p>If the real robot differs from what is written here, change it HERE and nowhere else.
 * Values marked {@code TODO-measure} are guesses that a student must measure on the real robot;
 * values marked {@code TODO-tune} come out of a tuning run. Values marked {@code (sim)} were
 * chosen so the ftc-sim robot in {@code TeamCode/robot-config/biobuzz.json} behaves; they are a
 * starting point only.</p>
 *
 * <p>Fields are {@code public static} (not final) so a live-tuning dashboard can change them
 * without a rebuild if the team adds one later.</p>
 */
public final class RobotConfig {
    private RobotConfig() {}

    // ------------------------------------------------------------------------------------------
    // 1. Drivetrain: mecanum, names and directions copied from BasicTeleop.java so the same
    //    Driver Hub robot configuration works for TeleOp and Auto.
    // ------------------------------------------------------------------------------------------
    public static String FRONT_LEFT = "fLeft";
    public static String FRONT_RIGHT = "fRight";
    public static String BACK_LEFT = "bLeft";
    public static String BACK_RIGHT = "bRight";

    public static DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    /** Drive motor encoder ticks per output-shaft revolution. 537.7 = goBILDA 5203 312 rpm. TODO-measure (check motor label). */
    public static double DRIVE_TICKS_PER_REV = 537.7;
    /** Mecanum wheel diameter in inches. 96 mm goBILDA mecanum = 3.78 in. TODO-measure. */
    public static double WHEEL_DIAMETER_IN = 96.0 / 25.4;
    /** Wheel revolutions per motor output revolution (1.0 = direct drive). TODO-measure. */
    public static double DRIVE_GEAR_RATIO = 1.0;
    /**
     * Mecanum wheels slip sideways: real strafe distance / what the encoders claim.
     * 0.8 matches ftc-sim's strafeEfficiency (robot-config/biobuzz.json). TODO-tune on the
     * real robot: strafe 48 in, divide measured distance by what telemetry reports.
     */
    public static double STRAFE_MULTIPLIER = 0.8;
    /** Same idea for forward distance; leave 1.0 unless forward 48 in reads wrong. TODO-tune. */
    public static double FORWARD_MULTIPLIER = 1.0;

    // ------------------------------------------------------------------------------------------
    // 2. Robot size (used for start poses against the wall). G304.F: starting config <= 18 in.
    // ------------------------------------------------------------------------------------------
    /** Front-to-back length in inches, bumper to bumper. TODO-measure. (sim robot: 0.40 m) */
    public static double ROBOT_LENGTH_IN = 0.40 / 0.0254;
    /** Side-to-side width in inches. TODO-measure. (sim robot: 0.40 m) */
    public static double ROBOT_WIDTH_IN = 0.40 / 0.0254;

    // ------------------------------------------------------------------------------------------
    // 3. IMU (Control Hub / Expansion Hub built-in). Name "imu" is the SDK default.
    // ------------------------------------------------------------------------------------------
    public static String IMU_NAME = "imu";
    /** TODO-measure: which way the REV logo on the hub faces when mounted on the robot. */
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    /** TODO-measure: which way the hub's USB ports face. */
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ------------------------------------------------------------------------------------------
    // 4. Localizer (how the robot knows where it is). Change ONE line to switch.
    // ------------------------------------------------------------------------------------------
    public enum LocalizerType {
        /** Drive-motor encoders + IMU heading. Works with no extra hardware, and in ftc-sim. Least accurate (wheels slip). */
        DRIVE_ENCODER_IMU,
        /** goBILDA Pinpoint odometry computer + 2 dead-wheel pods. Best accuracy. Fill in section 4a. */
        PINPOINT,
        /** SparkFun OTOS optical tracking sensor. Fill in section 4b. */
        OTOS
    }

    public static LocalizerType LOCALIZER = LocalizerType.DRIVE_ENCODER_IMU;

    // 4a. Pinpoint (only used when LOCALIZER == PINPOINT). Run the "Pinpoint Tuner" to fill these.
    public static String PINPOINT_NAME = "pinpoint";           // TODO-measure: name in robot config
    public static double PINPOINT_X_POD_OFFSET_IN = 0.0;       // TODO-tune (Pinpoint Tuner result)
    public static double PINPOINT_Y_POD_OFFSET_IN = 0.0;       // TODO-tune (Pinpoint Tuner result)
    public static boolean PINPOINT_X_POD_REVERSED = false;     // TODO-tune
    public static boolean PINPOINT_Y_POD_REVERSED = false;     // TODO-tune

    // 4b. OTOS (only used when LOCALIZER == OTOS). Run the "OTOS Tuner" to fill these.
    public static String OTOS_NAME = "otos";                   // TODO-measure: name in robot config
    public static double OTOS_OFFSET_X_IN = 0.0;               // TODO-tune (sensor position, +x forward)
    public static double OTOS_OFFSET_Y_IN = 0.0;               // TODO-tune (+y left)
    public static double OTOS_OFFSET_HEADING_DEG = 0.0;        // TODO-tune
    public static double OTOS_LINEAR_SCALAR = 1.0;             // TODO-tune
    public static double OTOS_ANGULAR_SCALAR = 1.0;            // TODO-tune

    // ------------------------------------------------------------------------------------------
    // 5. Mechanisms. Names are TODO: set them to whatever the team's Driver Hub config uses.
    //    If a device is missing from the hardware map, its subsystem becomes a safe no-op and the
    //    Auto still drives (and still LEAVEs / PARKs).
    // ------------------------------------------------------------------------------------------
    /** Intake roller: a motor OR a continuous-rotation servo with this name. TODO-measure. */
    public static String INTAKE_NAME = "intake";
    public static DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure
    public static double INTAKE_POWER = 1.0;

    /** Launcher flywheel motor (DcMotorEx). TODO-measure. */
    public static String LAUNCHER_NAME = "launcher";
    public static DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure
    /** Encoder ticks per flywheel motor revolution (28 = bare goBILDA/REV motor). TODO-measure. */
    public static double LAUNCHER_TICKS_PER_REV = 28.0;
    /** Optional second launcher device that pushes balls into the flywheel (motor or CR servo). TODO-measure; may not exist. */
    public static String FEEDER_NAME = "feeder";
    public static DcMotorSimple.Direction FEEDER_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure
    public static double FEEDER_POWER = 1.0;

    /**
     * Flywheel power for a shot from {@code FieldPoses.SHOOT_*}. (sim) 0.35 lands in the
     * upward CELL from 7.9 in in front of the audience wall; TODO-tune on the real field.
     */
    public static double LAUNCHER_POWER = 0.35;
    /** Seconds to wait after spinning up before the first ball goes in. (sim: flywheel 1.2 s) TODO-tune. */
    public static double LAUNCHER_SPINUP_S = 1.5;
    /** Ready when measured RPM is within this fraction of target (only if the motor has an encoder). */
    public static double LAUNCHER_READY_TOLERANCE = 0.05;
    /** Free speed of the flywheel motor, used to turn power into an expected RPM. (sim) TODO-measure. */
    public static double LAUNCHER_FREE_RPM = 6000.0;
    /** How long to keep feeding to empty the robot once the launcher is ready. TODO-tune. */
    public static double FIRE_DURATION_S = 2.0;

    // ------------------------------------------------------------------------------------------
    // 6. Autonomous timing (Competition Manual TU06 section 10.1: AUTO is 30 s).
    // ------------------------------------------------------------------------------------------
    public static double AUTO_LENGTH_S = 30.0;
    /**
     * When this many seconds are left, drop everything and go park. Must cover the longest park
     * drive in FieldPoses plus margin. (sim: worst park drive ~3 s)
     */
    public static double PARK_RESERVE_S = 6.0;
    /** Skip the optional GARDEN cycle unless at least this many seconds remain when it would start. */
    public static double CYCLE_MIN_TIME_S = 14.0;
    /** Max path speed (0..1) for the slow "creep" into the shooting spot. */
    public static double CREEP_SPEED = 0.25;
    /** Max path speed (0..1) while carrying collected POLLEN with the intake running. */
    public static double CARRY_SPEED = 0.6;
}
