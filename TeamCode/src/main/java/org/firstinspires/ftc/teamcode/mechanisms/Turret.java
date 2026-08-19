package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.control.PIDConstants;
import org.firstinspires.ftc.teamcode.control.StoppingDistance;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class Turret {
    // --- Tunable ---
    public static double staticGain = 0;
    public static PIDConstants pidConstants = new PIDConstants(0.02, 0.0, 0);
    public static Turret.ThresholdParameters THRESHOLD_PARAMETERS =
            new Turret.ThresholdParameters();
    public static double staticTheta = 0.0;
    public static double linearCoeff = 0.083; // (deg per deg/s)
    public static double quadCoeff = 0.000075; // (deg per (deg/s)^2)
    public static long staticThetaUpdateCounter = 0;
    public static double bias = 0;
    private static Turret instance;
    private static double prevAngle = 0.0;
    // --- Hardware constants ---
    private final double TICKS_PER_REV = 4000.0; // (ticks/rev) at the turret encoder
    private final double GEAR_RATIO = 140.0 / 30; // motor revs per turret rev
    // --- Hardware ---
    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final Encoder turretEncoder; // <-- replaced DcMotorEx with Encoder
    // --- Utilities ---
    private final PID pid;
    private final FtcDashboard dashboard;
    public boolean cutoff = false;
    double abcdefgh = 0;
    boolean set = false;
    double maxVelocity = 0;
    private double thetaConstant = 0.0;

    // --- Constructor ---
    private Turret(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        turretEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, "lfm"), this.TICKS_PER_REV
        );
        // <-- use Encoder wrapper

        dashboard = FtcDashboard.getInstance();
        pid = new PID(pidConstants, PID.functionType.LINEAR);
        turretLeft.setDirection(CRServo.Direction.REVERSE);
        turretRight.setDirection(CRServo.Direction.REVERSE);
        turretEncoder.reset();
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Turret(hardwareMap);
        staticTheta = 0;
        staticThetaUpdateCounter = 0;
    }

    public static Turret getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Shooter not initialized!");
        }
        return instance;
    }

    public void setInitialAngle(double angle) {
        thetaConstant = angle;
    }

    /**
     * @param velocity turret velocity (deg/s)
     *
     * @return angle the turret coasts through before stopping (deg)
     */
    public double computeStoppingDistance(double velocity) {
        return StoppingDistance.forVelocity(velocity, linearCoeff, quadCoeff);
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action
     */
    public Action setTurretAngle(double desiredAngle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (Math.abs(Utils.angleWrapDegrees(desiredAngle - getAngle())) < THRESHOLD_PARAMETERS.angleThreshold) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return false;
                }

                double power = computeSpinPower(Utils.angleWrapDegrees(desiredAngle));
                turretLeft.setPower(power);
                turretRight.setPower(power);

                //                packet.put("Target Angle", desiredAngle);
                //                packet.put("Current Angle", getAngle());
                //                packet.put("Power", power);
                return true;
            }
        };
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action
     */
    public Action setTurretAngleInfinite(double desiredAngle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!cutoff) {
                    if (Math.abs(Utils.angleWrapDegrees(desiredAngle - getAngle())) < THRESHOLD_PARAMETERS.angleThreshold) {
                        turretLeft.setPower(0);
                        turretRight.setPower(0);
                        return true;
                    }

                    double power = computeSpinPower(Utils.angleWrapDegrees(desiredAngle));
                    turretLeft.setPower(power);
                    turretRight.setPower(power);
                    //packet.put("Power", power);
                }

                //                packet.put("Target Angle", desiredAngle);
                //                packet.put("Current Angle", getAngle());
                return true;
            }
        };
    }

    public Action cutoffTurret() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                cutoff = true;
                return false;
            }
        };
    }

    public Action resumeTurret() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                cutoff = false;
                return false;
            }
        };
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action, TIMED
     */
    public Action setTurretAngleTimed(double desiredAngle, double seconds) {
        return new Action() {
            private double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                time = timer.seconds();
                if (timer.seconds() > seconds) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return false;
                }

                // Stop if angle reached
                if (Math.abs(Utils.angleWrapDegrees(desiredAngle - getAngle()))
                        < THRESHOLD_PARAMETERS.angleThreshold) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return true;
                }

                double power = computeSpinPower(Utils.angleWrapDegrees(desiredAngle));

                turretLeft.setPower(power);
                turretRight.setPower(power);

                return true;
            }
        };
    }

    //  --- Getter Functions ---

    /**
     * Manual turret control using gamepad stick
     *
     * @param stickPower (double) Gamepad Stick power
     */
    public Action manualControl(double stickPower) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turretLeft.setPower(stickPower);
                turretRight.setPower(stickPower);
                return true; // continuous
            }
        };
    }

    public Action stop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turretLeft.setPower(0);
                turretRight.setPower(0);
                return false; // continuous
            }
        };
    }

    /**
     * Computes PID power to reach a desired angle
     */
    /** Power toward desiredAngle [deg], by the shorter direction. */
    private double computeSpinPower(double desiredAngle) {
        double predictedAngle = getAngle() + computeStoppingDistance(getVelocity());
        double error = Utils.angleWrapDegrees(desiredAngle - predictedAngle);
        double pidOutput = pid.calculate(error, 0);
        return staticGain * Math.signum(pidOutput) + pidOutput;
    }


    /**
     * Returns the current bot-relative turret angle in degrees
     */
    public double getAngle() {
        double ticks = turretEncoder.getCurrentPosition(); // (ticks)
        double angleDeg = ((ticks / TICKS_PER_REV) * 360.0 / GEAR_RATIO); // (deg)
        return Utils.angleWrapDegrees(angleDeg + thetaConstant);
    }

    public void saveTheta(LinearOpMode op) {
        if (op.opModeIsActive()) {
            if (Math.abs(getAngle()) != 180) {
                staticTheta = getAngle();
                ++staticThetaUpdateCounter;
            }
        }
    }

    //    public void initAngle() {
    //        thetaConstant = prevAngle;
    //    }

    public Action saveAngle(LinearOpMode op) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                saveTheta(op);
                return true;
            }
        };
    }

    public Action saveAngleAndCount(LinearOpMode op) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //packet.put("count", staticThetaUpdateCounter);
                return saveAngle(op).run(packet);
            }
        };
    }

    // --- Auto-Aim Functions ---

    public Action saveAngleAndCountTimed(LinearOpMode op, double seconds) {
        return new Action() {
            private double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                //packet.put("count", staticThetaUpdateCounter);
                if (timer.seconds() < seconds) {
                    saveAngle(op).run(packet);
                    return true;
                }
                //packet.put("autoShootMoving Done", true);
                return false;
            }
        };
    }

    public void reset() {
        turretEncoder.reset();
    }

    /**
     * @return turret velocity (deg/s)
     */
    public double getVelocity() {
        return Math.toDegrees(turretEncoder.getVelocity()) / GEAR_RATIO;
    }

    /**
     * Auto-aim at a field goal using robot pose
     */
    public Action autoAim(Vector2d goalPos, double autoAimBias) {
        SimpleMatrix robotState = Drivetrain.getInstance().shootWhileMovingPose;
        Pose2d robotPose = new Pose2d(
                robotState.get(0, 0),
                robotState.get(1, 0),
                robotState.get(2, 0)
        );

        double angleToGoal = computeRobotRelativeAngle(robotPose, goalPos);

        return setTurretAngle(angleToGoal + autoAimBias);
    }

    /**
     * Loop to Auto-aim at a field goal using robot pose
     */
    public Action autoAimInfinite(Vector2d goalPos) {
        return new Action() {
            double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SimpleMatrix robotState = Drivetrain.getInstance().shootWhileMovingPose;
                Pose2d robotPose = new Pose2d(
                        robotState.get(0, 0),
                        robotState.get(1, 0),
                        robotState.get(2, 0)
                );
                double angleToGoal = computeRobotRelativeAngle(robotPose, goalPos);
                //                packet.put("angle to goal", angleToGoal);
                //                packet.put("angle", getAngle());
                return setTurretAngleInfinite(angleToGoal).run(packet);
            }
        };
    }

    public void setTurretPower(double power, boolean run, boolean reset, TelemetryPacket packet) {
        //        if (reset) {
        //            turretEncoder.reset();
        //        }
        if (Math.abs(this.getVelocity()) > maxVelocity) {
            maxVelocity = Math.abs(this.getVelocity());
        }
        if (run) {
            set = true;
            turretLeft.setPower(power);
            turretRight.setPower(power);
        } else if (set) {
            abcdefgh = getAngle();
            set = false;
            turretLeft.setPower(0);
            turretRight.setPower(0);
        } else {
            turretLeft.setPower(0);
            turretRight.setPower(0);
        }
        packet.put("max Velo", maxVelocity);
        packet.put("Current", getAngle());
        packet.put("Drifted", getAngle() - abcdefgh);
        packet.put("Stopped at", abcdefgh);
    }

    /**
     * Loop to Auto-aim at a field goal using robot pose
     */
    public Action autoAimTimed(Vector2d goalPos, double seconds) {
        return new Action() {
            double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                if (timer.seconds() < seconds) {
                    SimpleMatrix robotState = Drivetrain.getInstance().shootWhileMovingPose;
                    Pose2d robotPose = new Pose2d(
                            robotState.get(0, 0),
                            robotState.get(1, 0),
                            robotState.get(2, 0)
                    );
                    double angleToGoal = computeRobotRelativeAngle(robotPose, goalPos);
                    //                    packet.put("angle to goal", angleToGoal);
                    //                    packet.put("angle", getAngle());
                    //                    packet.put("autoShootMoving Done", true);
                    return setTurretAngleInfinite(angleToGoal).run(packet);
                }
                return false;
            }
        };
    }

    /**
     * Compute dx from robot to goal
     */
    private double computeDx(Pose2d robotPose, Vector2d goalPos) {
        return goalPos.x - robotPose.position.x;
    }

    /**
     * Compute dy from robot to goal
     */
    private double computeDy(Pose2d robotPose, Vector2d goalPos) {
        return goalPos.y - robotPose.position.y;
    }

    /**
     * Compute field-relative angle to goal in degrees
     */
    private double computeFieldAngle(double dx, double dy) {
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /**
     * Compute robot-relative angle to goal in degrees
     */
    private double computeRobotRelativeAngle(Pose2d robotPose, Vector2d goalPos) {
        double dx = computeDx(robotPose, goalPos);
        double dy = computeDy(robotPose, goalPos);
        double fieldAngle = computeFieldAngle(dx, dy);
        double robotHeading = Math.toDegrees(robotPose.heading.toDouble());
        double relativeAngle = fieldAngle - robotHeading;

        return Utils.angleWrapDegrees(relativeAngle);
    }

    public static class ThresholdParameters {

        public double angleThreshold = 2.0; // (deg)
    }
}