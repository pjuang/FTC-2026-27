package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Fallback AUTO that only scores LEAVE (3 points, TU06 10.5.4: "no longer contacting the perimeter
 * wall" at the end of AUTO).
 *
 * <p>Deliberately does NOT use Pedro Pathing, the IMU or any mechanism, so it works even before
 * anything is tuned and even if the intake/launcher are broken. It drives straight forward
 * {@link #DISTANCE_IN} inches using the drive encoders (or {@link #MAX_TIME_S} seconds, whichever
 * comes first, in case an encoder is unplugged), then stops. Works for either alliance from any
 * legal start that faces open field (G304). Make sure nothing is in front of the robot: no FLOWER,
 * no HIVE, no partner.</p>
 */
@Autonomous(name = "LeaveOnlyAuto", group = "BIOBUZZ", preselectTeleOp = "BasicTeleOp_Logitech")
public class LeaveOnlyAuto extends LinearOpMode {
    public static double DISTANCE_IN = 14.0;
    public static double POWER = 0.35;
    public static double MAX_TIME_S = 3.0;
    /** Wait this long after START, e.g. to let a partner go first. */
    public static double START_DELAY_S = 0.0;

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_LEFT);
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_RIGHT);
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_LEFT);
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_RIGHT);
        DcMotorEx[] motors = {fl, fr, bl, br};
        fl.setDirection(RobotConfig.FRONT_LEFT_DIRECTION);
        fr.setDirection(RobotConfig.FRONT_RIGHT_DIRECTION);
        bl.setDirection(RobotConfig.BACK_LEFT_DIRECTION);
        br.setDirection(RobotConfig.BACK_RIGHT_DIRECTION);
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double ticksPerInch = RobotConfig.DRIVE_TICKS_PER_REV
                / (Math.PI * RobotConfig.WHEEL_DIAMETER_IN * RobotConfig.DRIVE_GEAR_RATIO);
        double targetTicks = DISTANCE_IN * ticksPerInch;

        telemetry.addLine("LeaveOnlyAuto: drives forward " + DISTANCE_IN + " in. Clear path ahead?");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < START_DELAY_S) {
            idle();
        }
        timer.reset();
        for (DcMotorEx m : motors) m.setPower(POWER);
        double travelled = 0;
        while (opModeIsActive() && timer.seconds() < MAX_TIME_S) {
            travelled = 0;
            for (DcMotorEx m : motors) travelled += m.getCurrentPosition();
            travelled /= 4.0;
            if (travelled >= targetTicks) break;
            telemetry.addData("Travelled (in)", "%.1f / %.1f", travelled / ticksPerInch, DISTANCE_IN);
            telemetry.update();
        }
        for (DcMotorEx m : motors) m.setPower(0);
        telemetry.addData("Done. Travelled (in)", "%.1f", travelled / ticksPerInch);
        telemetry.update();
        while (opModeIsActive()) idle();   // stay still (G403)
    }
}
