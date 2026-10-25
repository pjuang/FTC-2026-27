package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ Auto, BLUE alliance, FAR start (see StartPosition). Strategy lives in BioBuzzAuto. */
@Autonomous(name = "BlueFarAuto", group = "BIOBUZZ", preselectTeleOp = "BasicTeleOp_Logitech")
public class BlueFarAuto extends BioBuzzAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected StartPosition startPosition() { return StartPosition.FAR; }
}
