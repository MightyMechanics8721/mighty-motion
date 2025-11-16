package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(group = "Tune", name = "TuneShooter")
public class ShooterTest extends LinearOpMode {
    public static double desiredVelocity = 2800;

    public static double kP = 0.005;

    public static double kV = 0.000215;
    
    DcMotorEx shooterLeft;
    DcMotorEx shooterRight;


    @Override
    public void runOpMode() throws InterruptedException {
//        Shooter shooter = new Shooter(hardwareMap);
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();


        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        packet.put("power", 0.0);
        packet.put("target velocity (RPM)", desiredVelocity);
        packet.put("current velocity (RPM)", 0.0);

        ftcDashboard.sendTelemetryPacket(packet);


        waitForStart();

        while (opModeIsActive()) {
//            Actions.runBlocking(shooter.motorSpin(desiredVelocity));

            double currentVelocity = shooterLeft.getVelocity() / 28.0 * 60.0;

            double power = kV * desiredVelocity + kP * (desiredVelocity - currentVelocity);

            shooterLeft.setPower(power);
            shooterRight.setPower(power);

            packet.put("power", power);
            packet.put("target velocity (RPM)", desiredVelocity);
            packet.put("current velocity (RPM)", currentVelocity);

            ftcDashboard.sendTelemetryPacket(packet);


        }


    }
}
