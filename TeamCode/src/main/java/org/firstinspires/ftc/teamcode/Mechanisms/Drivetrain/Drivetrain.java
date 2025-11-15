package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.inverseKinematics;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.l;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.r;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.w;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.makePoseVector;

import androidx.annotation.NonNull;

import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Util;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.DrivetrainMotorController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.GeometricController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.PoseController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Localizers.TwoWheelOdometery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Models.MecanumKinematicModel;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;


/**
 * Drivetrain class manages the robot's drive system, including motor control, odometry, and path
 * following. It provides methods for both autonomous and manual control, as well as telemetry
 * updates. This class is designed for use in FTC robots with mecanum or omni drive systems.
 * <p>
 * Inclues - Motor initialization and configuration - Odometry-based localization - Path following
 * and pose targeting - Manual control via controller inputs - Telemetry reporting for dashboard and
 * driver station
 */
@Config
public class Drivetrain {
    /**
     * The maximum Voltage the drivetrain could use at a time Used to save battery
     */
    public static double maxVoltage = 12.5;
    // Create new instance.
    public static PoseConstants POSE_CONSTANTS = new PoseConstants();
    public static FFConstantsController FF_CONSTANTS = new FFConstantsController();

    public SimpleMatrix state = new SimpleMatrix(6, 1);
    /**
     * Initialize Classes
     */
    public Battery battery;
    public TwoWheelOdometery twoWheelOdo;
    public DrivetrainMotorController motorController;
    public GeometricController geometricController;
    /**
     * Drivetrain motors
     */
    public DcMotorAdvanced motorLeftFront;
    public DcMotorAdvanced motorLeftBack;
    public DcMotorAdvanced motorRightBack;
    public DcMotorAdvanced motorRightFront;
    public SimpleMatrix wheelPowerPrev = new SimpleMatrix(4, 1);
    public SimpleMatrix prevWheelSpeeds = new SimpleMatrix(new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public SimpleMatrix stopMatrix = new SimpleMatrix(new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public PoseController poseControl = new PoseController(
            POSE_CONSTANTS.xPIDConstants,
            POSE_CONSTANTS.yPIDConstants,
            POSE_CONSTANTS.headingPIDConstants
    );
    MechanicalParameters MECHANICAL_PARAMETERS = new MechanicalParameters();

    ThresholdParameters THRESHOLD_PARAMETERS = new ThresholdParameters();

    MotionParameters MOTION_PARAMETERS = new MotionParameters();
    HardwareMap hardwareMap;
    SimpleMatrix initialState = new SimpleMatrix(6, 1);
    FtcDashboard ftcDashboard;
    DebuggingParameters DEBUG = new DebuggingParameters();
    private MecanumKinematicModel mecanumKinematicModel;

    /**
     * Initializes the Drivetrain (Wheels of the Robot)
     *
     * @param hardwareMap The hardwareMap of the Robot, describes which port of the hub is connected
     * to which name
     * @param battery The Battery level of the Robot
     */
    public Drivetrain(HardwareMap hardwareMap, Battery battery) {
        this.hardwareMap = hardwareMap;
        this.motorController = new DrivetrainMotorController(hardwareMap, FF_CONSTANTS);
        this.twoWheelOdo = new TwoWheelOdometery(hardwareMap);
        this.geometricController = new GeometricController();
        this.mecanumKinematicModel = new MecanumKinematicModel(MECHANICAL_PARAMETERS);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        this.motorLeftFront = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "lfm"),
                battery,
                maxVoltage
        );
        this.motorLeftBack = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "lbm"),
                battery,
                maxVoltage
        );
        this.motorRightBack = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "rbm"),
                battery,
                maxVoltage
        );
        this.motorRightFront = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "rfm"),
                battery,
                maxVoltage
        );

        this.motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        /**
         * Establish that motors will not be using their native encoders:
         * 'RUN_WITHOUT_ENCODER' does not actually run without encoders, it
         * deactivates the PID
         */
        this.motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorLeftFront.setPower(0);
        this.motorLeftBack.setPower(0);
        this.motorRightFront.setPower(0);
        this.motorRightBack.setPower(0);

        this.ftcDashboard = FtcDashboard.getInstance();

        this.twoWheelOdo.resetPosAndRecalibrateIMU();
    }

    /**
     * Sets the Position of the bot in its start position.
     *
     * @param x Initial X position (inches)
     * @param y Initial Y position (inches)
     * @param theta Initial heading (radians)
     */
    public void setInitialPosition(double x, double y, double theta) {
        initialState.set(0, 0, x);
        initialState.set(1, 0, y);
        initialState.set(2, 0, theta);
    }

    /**
     * Localizes the Robot, determines the current location of the Robot using odometry and previous
     * locations. Updates the internal state matrix with the current estimated pose.
     */
    public void localize() {
        this.state = this.initialState.plus(this.twoWheelOdo.calculate());
        this.updateTelemetry();
    }


    /**
     * Sets the power to the wheels & records Previous Power. Only updates power if the change
     * exceeds acceptablePowerDifference to save battery.
     *
     * @param powers matrix of wheel power values (order:lfm, lbm, rbm, rfm)
     */
    public void setPower(SimpleMatrix powers) {
        double u0 = powers.get(0, 0);
        double u1 = powers.get(1, 0);
        double u2 = powers.get(2, 0);
        double u3 = powers.get(3, 0);
        double u0Prev = wheelPowerPrev.get(0, 0);
        double u1Prev = wheelPowerPrev.get(1, 0);
        double u2Prev = wheelPowerPrev.get(2, 0);
        double u3Prev = wheelPowerPrev.get(3, 0);
        motorLeftFront.setPower(powers.get(0, 0));
        motorLeftBack.setPower(powers.get(1, 0));
        motorRightBack.setPower(powers.get(2, 0));
        motorRightFront.setPower(powers.get(3, 0));
        wheelPowerPrev.set(0, 0, u0);
        wheelPowerPrev.set(1, 0, u1);
        wheelPowerPrev.set(2, 0, u2);
        wheelPowerPrev.set(3, 0, u3);
    }

    /**
     * Sets the Wheels speed and acceleration.
     *
     * @param wheelSpeeds Current Wheel Speed
     * @param wheelAccelerations Increment of Wheel Speed
     */
    public void setWheelSpeedAcceleration(
            SimpleMatrix wheelSpeeds,
            SimpleMatrix wheelAccelerations
    ) {
        setPower(motorController.calculate(wheelSpeeds, wheelAccelerations));
    }

    private double stoppingDistanceX(double xVelocity) {
        return 0.0932 * xVelocity + 0.00124 * xVelocity * xVelocity;
    }

    private double stoppingDistanceY(double yVelocity) {
        return 0.0832 * yVelocity + 0.00149 * yVelocity * yVelocity;
    }

    /**
     * Moves the robot to a desired pose using PID control.
     *
     * @param desiredPose The target pose [x, y, theta] in field coordinates.
     *
     * @return An Action that runs until the robot is within distanceThreshold and angleThreshold of
     * the target.
     */
    public Action goToPose(
            SimpleMatrix desiredPose, double distanceThreshold,
            double angleThreshold
    ) {
        Drivetrain drivetrain = this;

        //Rename goToPosition
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                localize();
                SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);


                // 1) Get the x velocity and the y velocity
                // 2) Plug in each velocity to each function to get the stopping distance of x
                // and of y
                // 3) Build a new simple matrix that is 3x1 that contains [x stop dist, y stop
                // dist, 0]

                SimpleMatrix stopDistance = new SimpleMatrix(
                        new double[][]{
                                new double[]{
                                        Math.signum(state.get(3, 0)) * drivetrain.stoppingDistanceX(
                                                Math.abs(state.get(3, 0)))
                                },
                                new double[]{
                                        Math.signum(state.get(4, 0)) * drivetrain.stoppingDistanceY(
                                                Math.abs(state.get(4, 0)))
                                },
                                new double[]{0}
                        }
                );
                // 4) Rotate this matrix to the global frame
                SimpleMatrix stopDistanceGlobal = Utils.rotateBodyToGlobal(
                        stopDistance, state.get(
                                2,
                                0
                        )
                );
                // 5) Add the result to the pose
                if (ThresholdParameters.stopPlanning) pose = pose.plus(stopDistanceGlobal);
                SimpleMatrix wheelSpeeds
                        = mecanumKinematicModel.inverseKinematics(poseControl.calculate(
                        pose,
                        desiredPose
                ));
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                //                deltaT.reset();
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;
                if (Math.abs(Utils.calculateDistance(
                        state.get(0, 0),
                        state.get(1, 0),
                        desiredPose.get(0, 0),
                        desiredPose.get(1, 0)
                )) < distanceThreshold
                        && Math.abs(Utils.angleWrap(state.get(2, 0) - desiredPose.get(2, 0)))
                        < angleThreshold) {
                    setPower(stopMatrix);
                    packet.put("Done", "done");
                }
                return !(Math.abs(Utils.calculateDistance(
                        state.get(0, 0),
                        state.get(1, 0),
                        desiredPose.get(0, 0),
                        desiredPose.get(1, 0)
                )) < ThresholdParameters.distanceThreshold
                        && Math.abs(Utils.angleWrap(state.get(2, 0) - desiredPose.get(2, 0)))
                        < angleThreshold);
            }
        };
    }

    public Action goToPose(
            SimpleMatrix desiredPose
    ) {
        return this.goToPose(
                desiredPose, this.THRESHOLD_PARAMETERS.distanceThreshold,
                this.THRESHOLD_PARAMETERS.angleThreshold
        );
    }

    /**
     * Stops the Motors of the drivetrain immediately.
     *
     * @return An InstantAction that sets all wheel powers to zero.
     */
    public InstantAction stopMotors() {
        return new InstantAction(() -> setPower(stopMatrix));
    }

    /**
     * Follows a given path using geometric control and a motion profile. Scales wheel speeds based
     * on the path's velocity profile.
     *
     * @param path The Path object to follow.
     *
     * @return An Action that runs until the robot reaches the end of the path within
     * distanceThreshold.
     */
    public Action followPath(Path path) {
        //        FtcDashboard ftcDashboard = FtcDashboard.getInstance();

        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                localize();
                if ((Math.abs(Utils.calculateDistance(
                        state.get(0, 0),
                        state.get(1, 0),
                        path.getFinalPoint()[0],
                        path.getFinalPoint()[1]
                )) < 12)) {
                    packet.addLine("USING POSE CONTROLLER");
                    return goToPose(makePoseVector(
                            path.getFinalPoint()[0], path.getFinalPoint()[1],
                            path.finalHeading
                    )).run(packet);
                }
                packet.addLine("USING GEO CONTROLLER");
                SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);

                SimpleMatrix desiredPose = geometricController.calculate(state, path);
                SimpleMatrix wheelSpeeds =
                        mecanumKinematicModel.inverseKinematics(poseControl.calculate(
                                pose,
                                desiredPose
                        ));

                double maxScale = (MOTION_PARAMETERS.maxSpeed / MECHANICAL_PARAMETERS.wheelRadius)
                        / wheelSpeeds.elementMaxAbs();

                wheelSpeeds = wheelSpeeds.scale(maxScale);
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);

                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;

                return !(Math.abs(Utils.calculateDistance(
                        state.get(0, 0),
                        state.get(1, 0),
                        desiredPose.get(0, 0),
                        desiredPose.get(1, 0)
                )) < ThresholdParameters.distanceThreshold
                        && Math.abs(Utils.angleWrap(state.get(2, 0) - desiredPose.get(2, 0)))
                        < ThresholdParameters.angleThreshold);
            }
        };
    }

    public void updateTelemetry() {
        if (!DebuggingParameters.debug) return;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x position", state.get(0, 0));
        packet.put("y position", state.get(1, 0));
        packet.put("heading", Math.toDegrees(state.get(2, 0)));
        packet.put("longitudinal Velocity", state.get(3, 0));
        packet.put("lateral Velocity", state.get(4, 0));
        packet.put("heading Velocity", state.get(5, 0));

        // Draw robot
        Canvas canvas = packet.fieldOverlay();
        Drawing.drawRobot(canvas, state);

        ftcDashboard.sendTelemetryPacket(packet);
    }

    /**
     * Allows for manual control of Robot using controller joystick.
     *
     * @param ly Left stick Y axis (forward/backward)
     * @param lx Left stick X axis (strafe left/right)
     * @param rX Right stick X axis (rotation)
     *
     * @return An Action that applies the joystick values to the drivetrain for manual driving.
     */
    public Action manualControl(double ly, double lx, double rX) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double y = ly;
                double x = -lx;
                double rx = -rX;
                SimpleMatrix compensatedTwist = new SimpleMatrix(
                        new double[][]{
                                new double[]{MechanicalParameters.wheelRadius * x},
                                new double[]{MechanicalParameters.wheelRadius * y},
                                new double[]{
                                        (MechanicalParameters.wheelRadius / (
                                                MechanicalParameters.longDistToAxles
                                                        + MechanicalParameters.latDistToAxles)) * rx
                                },
                                }
                );
                double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
                setPower(mecanumKinematicModel.inverseKinematics(compensatedTwist)
                                              .scale(1 / denominator));
                telemetryPacket.put("X", state.get(0, 0));
                telemetryPacket.put("Y", state.get(1, 0));
                telemetryPacket.put("Theta", Math.toDegrees(state.get(2, 0)));
                return false;
            }
        };
    }

    public static class MotionParameters {
        public double maxSpeed = 80.0; // (in/s)
    }

    public static class PoseConstants {
        public PIDConstants xPIDConstants = new PIDConstants(30, 0, 0);

        public PIDConstants yPIDConstants = new PIDConstants(35, 0, 0);

        public PIDConstants headingPIDConstants = new PIDConstants(6, 0, 0);
    }

    public static class FFConstantsController {
        public FFConstants lf = new FFConstants(0.1, 0.0225, 0.067);
        public FFConstants lb = new FFConstants(0.1, 0.0225, 0.067);
        public FFConstants rb = new FFConstants(0.1, 0.0225, 0.067);
        public FFConstants rf = new FFConstants(0.1, 0.0225, 0.067);
    }

    public static class MechanicalParameters {
        public static double wheelRadius = 2.16535; // (in)
        public static double longDistToAxles = 5.7;
        // (in) Longitudinal distance from center to axles
        public static double latDistToAxles = 5.31496; // (in) Lateral distance from center to axles
    }

    /**
     * Debugging parameters for the Drivetrain. Can turn telemetry printouts on/off globally.
     */
    public static class DebuggingParameters {
        /**
         * Set to true to enable telemetry printouts, false to disable
         */
        public static boolean debug = true;
    }


    public static class ThresholdParameters {
        /**
         * Acceptable difference between current and previous wheel power to make a hardware call
         * Used to save battery
         */
        public static double acceptablePowerDifference = 0.000001;
        /**
         * Acceptable difference between wanted and current positions (Inches) to make a hardware
         * call Used to save time & reduce unnecessary movements
         */
        public static double distanceThreshold = 0.25;
        /**
         * The acceptable difference between wanted and current angles (Radians) Used to save time &
         * reduce unnecessary movements
         */
        public static double angleThreshold = 0.1;
        public static boolean stopPlanning = true;
    }
}
