package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

// Teleop boilerplate setup code
@TeleOp(name = "BasicTeleOp_Logitech")
public class BasicTeleop extends LinearOpMode {
    public static double brake(double a, double b) { // b 0.0 - 1.0, a -1.0 - 1.0
        if (a == 0 || Math.abs(a) < b) {
            return 0.0;
        }
        return (a > 0) ? a - b : a + b; // ternary, a-b if a>0 and vice versa
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors (0,1,2,3)
        DcMotor frontLeft  = hardwareMap.dcMotor.get("fLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("fRight");
        DcMotor backLeft   = hardwareMap.dcMotor.get("bLeft");
        DcMotor backRight  = hardwareMap.dcMotor.get("bRight");

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set directions so pushing forward goes forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        double brakeAmount = 0.5;
        double turnBrakeAmount = 0.5;

        double drive = 0;
        double turn = 0;
        double strafe = 0;

        while (opModeIsActive()) {
            // Debugging
            telemetry.addLine("Left_stick_y: " + gamepad1.left_stick_y);
            telemetry.addLine("Right_trigger: " + gamepad1.right_trigger);
            telemetry.addLine("Right bumper: " + gamepad1.right_bumper);
            telemetry.addLine("Left bumper: " + gamepad1.left_bumper);
            telemetry.addLine("Brake amount: " + brakeAmount);
            telemetry.addLine("Turn brake amount: " + turnBrakeAmount);

            // Logitech gamepad:
            // Left stick Y → drive
            // Right stick X → turn
            // Right bumper → Brake by brakeAmount
            // Left bumper → Turn brake by turnBrakeAmount(slow turn)
            // Right trigger → Brake by trigger amount
            // Left trigger → Turn brake by trigger amount

            // Driving
            if(gamepad1.right_bumper) {
                drive = brake(-gamepad1.left_stick_y, brakeAmount);
                strafe = brake(gamepad1.left_stick_x, brakeAmount);

            }
            else{
                drive = brake(-gamepad1.left_stick_y, gamepad1.right_trigger);
                strafe = brake(gamepad1.left_stick_x, gamepad1.right_trigger);
            }

            // Turning
            if(gamepad1.left_bumper) {
                turn = brake(gamepad1.right_stick_x, turnBrakeAmount);
            }
            else{
                turn = brake(gamepad1.right_stick_x, gamepad1.left_trigger);
            }

            // Dpad driving
            if(gamepad1.dpad_up){
                if(gamepad1.right_bumper) {
                    drive = brake(1, brakeAmount);
                }
                else{
                    drive = brake(1, gamepad1.right_trigger);
                }
            } else if (gamepad1.dpad_down) {
                if(gamepad1.right_bumper) {
                    drive = brake(-1, brakeAmount);
                }
                else{
                    drive = brake(-1, gamepad1.right_trigger);
                }
            } if (gamepad1.dpad_left){
                if(gamepad1.right_bumper) {
                    strafe = brake(-1, brakeAmount);
                }
                else{
                    strafe = brake(-1, gamepad1.right_trigger);
                }
            } else if (gamepad1.dpad_right){
                if(gamepad1.right_bumper) {
                    strafe = brake(1, brakeAmount);
                }
                else{
                    strafe = brake(1, gamepad1.right_trigger);
                }
            }

            // DPAD2 for precise forward
            // 2 brakes, one for set clamp speed one for trigger hold amount

            // Omni-directional movement pattern
            double fl = drive + turn + strafe;
            double fr = drive - turn - strafe;
            double bl = drive + turn - strafe;
            double br = drive - turn + strafe;

            // Normalize power values
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;

            // Set power
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);

            // Telemetry
            telemetry.addData("drive", drive);
            telemetry.addData("turn", turn);
            telemetry.update();
        }
    }
}
