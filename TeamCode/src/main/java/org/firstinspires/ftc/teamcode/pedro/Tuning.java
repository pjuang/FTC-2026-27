package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing v3 tuners (the v3 replacement for RobToT's {@code tuners_tests} OpModes).
 *
 * <p>Each {@code @Tuner} method shows up in Pedro's tuning web page, which the
 * {@code com.pedropathing:tuning} library serves from the Control Hub while the robot app is
 * running (library source: WebServer on port 10158). Connect a laptop to the robot's Wi-Fi and open
 * {@code http://192.168.43.1:10158}. Run them in this order:</p>
 * <ol>
 *   <li>Mecanum Tuner: checks motor directions (compare with RobotConfig).</li>
 *   <li>Pinpoint Tuner or OTOS Tuner: only if RobotConfig.LOCALIZER uses that sensor.</li>
 *   <li>Foresight Tuner: measures velocities/decelerations and tunes the controllers, then prints
 *       Java to paste over {@code Constants.foresightConfig}.</li>
 *   <li>Tests: localization, line, curve and hold tests using {@link Constants}.</li>
 * </ol>
 * <p>The procedure classes in {@code pedro/procedures} are copied unchanged from the official
 * Pedro Quickstart (commit 9dc8978, 2026-10-22).</p>
 */
public class Tuning {

    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2a. Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "2b. OTOS Tuner")
    public static Procedure otos() {
        return new OTOSTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }
}
