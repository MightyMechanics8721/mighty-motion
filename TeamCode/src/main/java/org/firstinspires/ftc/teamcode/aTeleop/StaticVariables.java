package org.firstinspires.ftc.teamcode.aTeleop;

import org.ejml.simple.SimpleMatrix;

public class StaticVariables {
    public static double staticTurretAngle;
    public static SimpleMatrix staticRobotState;


//    public static Action updateTurretAngle(boolean end1, boolean end2) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//                double turretAngle = Turret.getInstance().getAngle();
//
//
//                if (end1 && !end2) {
//                    staticTurretAngle = turretAngle;
//                }
//                return true;
//            }
//        };
//    }
//
//    public static Action updateRobotState(boolean end1, boolean end2) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//                SimpleMatrix robotState = Drivetrain.getInstance().state;
//                if (end1 && !end2) {
//                    staticRobotState = robotState;
//                }
//                return true;
//            }
//        };
//    }


}
