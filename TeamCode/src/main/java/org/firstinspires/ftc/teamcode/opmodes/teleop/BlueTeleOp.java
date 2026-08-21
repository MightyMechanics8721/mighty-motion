package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Alliance;

/** Driver-controlled routine, blue alliance. Everything lives in {@link DriverControlled}. */
@TeleOp(name = "Blue TeleOp", group = "1123Competition")
public class BlueTeleOp extends DriverControlled {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
