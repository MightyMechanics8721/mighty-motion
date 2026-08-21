package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Alliance;

/** Driver-controlled routine, red alliance. Everything lives in {@link DriverControlled}. */
@TeleOp(name = "Red TeleOp", group = "1123Competition")
public class RedTeleOp extends DriverControlled {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
