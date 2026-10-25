package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ Auto, BLUE alliance, NEAR start (see StartPosition). Strategy lives in BioBuzzAuto. */
@Autonomous(name = "BlueAuto", group = "BIOBUZZ", preselectTeleOp = "BasicTeleOp_Logitech")
public class BlueAuto extends BioBuzzAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected StartPosition startPosition() { return StartPosition.NEAR; }
}
