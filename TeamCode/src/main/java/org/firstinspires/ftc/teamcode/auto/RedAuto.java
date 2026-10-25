package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ Auto, RED alliance, NEAR start (see StartPosition). Strategy lives in BioBuzzAuto. */
@Autonomous(name = "RedAuto", group = "BIOBUZZ", preselectTeleOp = "BasicTeleOp_Logitech")
public class RedAuto extends BioBuzzAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected StartPosition startPosition() { return StartPosition.NEAR; }
}
