package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Intake roller. Works with either a DC motor or a continuous-rotation servo named
 * {@link RobotConfig#INTAKE_NAME}; if neither exists every method is a safe no-op.
 */
public class Intake {
    private final DcMotorSimple roller;   // DcMotor and CRServo are both DcMotorSimple
    private double power;

    public Intake(HardwareMap hardwareMap) {
        roller = OptionalHardware.get(hardwareMap, DcMotorSimple.class, RobotConfig.INTAKE_NAME);
        if (roller != null) {
            roller.setDirection(RobotConfig.INTAKE_DIRECTION);
            if (roller instanceof DcMotor) {
                ((DcMotor) roller).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            roller.setPower(0);
        }
    }

    public boolean isPresent() { return roller != null; }

    /** Pull game elements in. */
    public void in() { set(RobotConfig.INTAKE_POWER); }

    /** Push game elements out. */
    public void out() { set(-RobotConfig.INTAKE_POWER); }

    public void stop() { set(0); }

    public void set(double newPower) {
        power = newPower;
        if (roller != null) roller.setPower(newPower);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", roller == null ? "MISSING (no-op): '" + RobotConfig.INTAKE_NAME + "'"
                : String.format("power %.2f", power));
    }
}
