package org.firstinspires.ftc.teamcode.Mechanisms.Turret;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class Turret {
    // --- Tunable ---
    public static double staticGain = 0.1;
    public static PIDConstants pidConstants = new PIDConstants(0.01, 0.0, 0);
    public static double angleThreshold = 1.0;
    public static Turret.ThresholdParameters THRESHOLD_PARAMETERS =
            new Turret.ThresholdParameters();
    public static double turretAngle = 0;
    public static double staticTheta = 0.0;
    public static double linearCoeff = 0.075;
    public static double quadCoeff = 0.000075;
    public static long staticThetaUpdateCounter = 0;
    public static double bias = 0;
    private static Turret instance;
    private static double prevAngle = 0.0;
    // --- Hardware constants ---
    private final double TICKS_PER_REV = 4000.0;
    private final double GEAR_RATIO = 140.0 / 30;
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
        //        turretAngle = 0;
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Turret(hardwareMap);
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

    public double computeStoppingDistance(double velocity) {
        return Math.signum(velocity) * addDrift(Math.abs(
                velocity));
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action
     */
    public Action setTurretAngle(double desiredAngle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (Math.abs(desiredAngle - getAngle()) < THRESHOLD_PARAMETERS.angleThreshold) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return false;
                }

                double power = computeSpinPower(clamp(desiredAngle, -180, 180));
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
                    if (Math.abs(desiredAngle - getAngle()) < THRESHOLD_PARAMETERS.angleThreshold) {
                        turretLeft.setPower(0);
                        turretRight.setPower(0);
                        return true;
                    }


                    double power = computeSpinPower(clamp(desiredAngle, -180, 180));
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
                if (Math.abs(desiredAngle - getAngle())
                        < THRESHOLD_PARAMETERS.angleThreshold) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return true;
                }

                double power = computeSpinPower(clamp(desiredAngle, -180, 180));

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
    private double computeSpinPower(double desiredAngle) {
        double velocity = getVelocity();
        double pidOutput = pid.calculate(
                desiredAngle,
                getAngle() + computeStoppingDistance(velocity)
        );
        return staticGain * Math.signum(pidOutput) + pidOutput;
    }

    private double addDrift(double vel) {
        return linearCoeff * vel + quadCoeff * Math.pow(vel, 2);
    }

    /**
     * Returns the current bot-relative turret angle in degrees
     */
    public double getAngle() {
        double ticks = turretEncoder.getCurrentPosition();
        double angleDeg = ((ticks / TICKS_PER_REV) * 360.0 / GEAR_RATIO);
        return (angleDeg + thetaConstant) % 360;
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
     * Returns the current turret velocity in degrees/sec
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

        // Wrap 0–360
        //        return (relativeAngle % 360 + 360) % 360;
        return Math.toDegrees(Utils.angleWrap(Math.toRadians(relativeAngle)));
    }

    public static class ThresholdParameters {

        /**
         * Maximum expected voltage of the battery in volts.
         */
        public double angleThreshold = 2.0; // (deg)
    }
}