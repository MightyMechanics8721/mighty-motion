package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.inverseKinematics;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.l;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.r;
//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.w;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.makePoseVector;

import androidx.annotation.NonNull;

import java.util.Arrays;
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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static PoseConstantsGeo POSE_CONSTANTS_GEO = new PoseConstantsGeo();
    public static FFConstantsController FF_CONSTANTS = new FFConstantsController();
    public static MechanicalParameters mechanicalParameters;
    public static MechanicalParameters MECHANICAL_PARAMETERS = new MechanicalParameters();
    public static ThresholdParameters THRESHOLD_PARAMETERS = new ThresholdParameters();
    public static MotionParameters MOTION_PARAMETERS = new MotionParameters();
    public SimpleMatrix stoppingDistancePose = new SimpleMatrix(3, 1);
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

    public PoseController followController = new PoseController(
            POSE_CONSTANTS_GEO.xPIDConstants,
            POSE_CONSTANTS_GEO.yPIDConstants,
            POSE_CONSTANTS_GEO.headingPIDConstants
    );
    HardwareMap hardwareMap;
    SimpleMatrix initialState = new SimpleMatrix(6, 1);
    FtcDashboard ftcDashboard;
    DebuggingParameters DEBUG = new DebuggingParameters();
    private MecanumKinematicModel mecanumKinematicModel;

    /**
     * Initializes the Drivetrain (Wheels of the Robot)
     *
     * @param hardwareMap The hardwareMap of the Robot, describes which port of the hub is connected
     *                    to which name
     * @param battery     The Battery level of the Robot
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
     * @param x     Initial X position (inches)
     * @param y     Initial Y position (inches)
     * @param theta Initial heading (degrees)
     */
    public void setInitialPose(double x, double y, double theta) {
        this.twoWheelOdo.odo.setPosX(x, DistanceUnit.INCH);
        this.twoWheelOdo.odo.setPosY(y, DistanceUnit.INCH);
        this.twoWheelOdo.odo.setHeading(theta, AngleUnit.DEGREES);


        // TODO: remove
        this.localize();
        stoppingDistancePose = state.extractMatrix(0, 3, 0, 1);
    }

    /**
     * Localizes the Robot, determines the current location of the Robot using odometry and previous
     * locations. Updates the internal state matrix with the current estimated pose.
     */
    public void localize(TelemetryPacket packet) {
        this.localize();
        this.updateTelemetry(packet);
    }

    public void localize() {
        this.state = this.twoWheelOdo.calculate();
    }

    //    public void localizePath(double[][] points, TelemetryPacket packet) {
    //        this.state = this.initialState.plus(this.twoWheelOdo.calculate());
    //        //        this.updateTelemetryGraph(points, packet);
    //    }


    public SimpleMatrix stoppingDistance(TelemetryPacket packet) {
        // 1) Get the x velocity and the y velocity
        // 2) Plug in each velocity to each function to get the stopping distance of x
        // and of y
        // 3) Build a new simple matrix that is 3x1 that contains [x stop dist, y stop
        // dist, 0]
        //        Drivetrain drivetrain = this;
        SimpleMatrix stopDistance = new SimpleMatrix(
                new double[][]{
                        new double[]{
                                Math.signum(state.get(3, 0)) * this.stoppingDistanceX(
                                        Math.abs(state.get(3, 0)))
                        },
                        new double[]{
                                Math.signum(state.get(4, 0)) * this.stoppingDistanceY(
                                        Math.abs(state.get(4, 0)))
                        },
                        new double[]{
                                Math.signum(state.get(5, 0)) * this.stoppingAngle(
                                        Math.abs(state.get(5, 0))
                                )
                        }
                }
        );
        // 4) Rotate this matrix to the global frame
        SimpleMatrix stopDistanceGlobal = Utils.rotateBodyToGlobal(
                stopDistance, state.get(
                        2,
                        0
                )
        );

        packet.put("stopping x", stopDistance.get(0, 0));
        packet.put("stopping y", stopDistance.get(1, 0));
        packet.put("stopping angle", stopDistance.get(2, 0));

        return stopDistanceGlobal;
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
     * @param wheelSpeeds        Current Wheel Speed
     * @param wheelAccelerations Increment of Wheel Speed
     */
    public void setWheelSpeedAcceleration(
            SimpleMatrix wheelSpeeds,
            SimpleMatrix wheelAccelerations
    ) {
        setPower(motorController.calculate(wheelSpeeds, wheelAccelerations));
    }

    private double stoppingDistanceX(double xVelocity) {
        return 0.132 * xVelocity + 0.00132 * xVelocity * xVelocity;
    }

    private double stoppingDistanceY(double yVelocity) {
        return 0.0716 * yVelocity + 0.00213 * yVelocity * yVelocity;
    }

    private double stoppingAngle(double angularVelocity) {
        return 0.0658 * angularVelocity + 0.00522 * angularVelocity * angularVelocity;
    }

    private double distance(
            double[] desiredPos, double distanceThreshold,
            TelemetryPacket packet
    ) {
        double distanceToGoal = Utils.calculateDistance(
                state.get(0, 0),
                state.get(1, 0),
                desiredPos[0],
                desiredPos[1]
        );
        packet.put("dist. to goal/final point (in)", distanceToGoal);
        packet.put("dist. thresh (in)", distanceThreshold);
        return distanceToGoal;
    }

    private boolean inHeading(double heading, double angleThreshold, TelemetryPacket packet) {
        double headingError = Utils.angleWrap(heading - state.get(2, 0));
        packet.put("heading error (deg)", Math.toDegrees(headingError));
        packet.put("angle thresh (deg)", Math.toDegrees(angleThreshold));
        return Math.abs(headingError) <= Math.abs(angleThreshold);
    }

    private boolean inStoppingZone(
            SimpleMatrix desiredPose, double distanceThreshold,
            double angleThreshold,
            TelemetryPacket packet
    ) {
        double[] position = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        return (Math.abs(this.distance(position, distanceThreshold, packet)) <= Math.abs(
                distanceThreshold)) && this.inHeading(
                desiredPose.get(2, 0),
                angleThreshold,
                packet
        );
    }

    private boolean goToPoseFunction(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance,
            TelemetryPacket packet
    ) {
        double[] desiredPosition = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        Canvas canvas = packet.fieldOverlay();
        Drawing.drawPoint(desiredPosition, canvas, "green");

        SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);
        if (useStoppingDistance) {
            pose =
                    pose.plus(this.stoppingDistance(packet));
            packet.addLine("stop dist. used");
        }
        stoppingDistancePose = pose;

        SimpleMatrix wheelSpeeds
                = mecanumKinematicModel.inverseKinematics(poseControl.calculate(
                pose,
                desiredPose
        ));

        SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);

        setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);

        boolean readyToStop = this.inStoppingZone(
                desiredPose,
                distanceThreshold,
                angleThreshold,
                packet
        );
        if (readyToStop) {
            setPower(stopMatrix);
            packet.addLine("robot within pose thresh.");
        }

        Drawing.drawRobot(stoppingDistancePose, canvas, "blue");
        return !readyToStop;
    }

    public Action goToPose(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance
    ) {
        Drivetrain drivetrain = this;
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drivetrain.localize(packet);

                return drivetrain.goToPoseFunction(
                        desiredPose, distanceThreshold,
                        angleThreshold,
                        useStoppingDistance,
                        packet
                );
            }
        };

    }

    // TODO: FIXME: REMOVE THIS!
    public Action goToPose(
            SimpleMatrix simpl
    ) {
        Drivetrain drivetrain = this;
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return true;
            }
        };

    }

    // TODO: so we can upload. REMOVE LATER
    public Action goToPose(
            SimpleMatrix simpl, double x, double y
    ) {
        Drivetrain drivetrain = this;
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return true;
            }
        };

    }

    private boolean followPathFunction(
            Path path, double maxSpeed, double distanceThreshold, double angleThreshold
            , boolean useStoppingDistance, TelemetryPacket packet
    ) {
        double[] position = {state.get(0, 0), state.get(1, 0)};

        Canvas canvas = packet.fieldOverlay();
        Drawing.drawPath(path.getWaypoints(), canvas, "red");
        Drawing.drawCircle(position, canvas, GeometricController.lookAheadXY, "purple", false);
        Drawing.drawCircle(position, canvas, GeometricController.lookAheadTheta, "orange", false);

        SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);
        double distanceToFinalPose = distance(
                path.getFinalPoint(), distanceThreshold,
                packet
        );
        packet.put("dist. to final point (in)", distanceToFinalPose);

        if (Math.abs(distanceToFinalPose) <= GeometricController.lookAheadTheta) {
            packet.addLine("follower: pose control");

            SimpleMatrix desiredPose = makePoseVector(
                    path.getFinalPoint()[0], path.getFinalPoint()[1],
                    path.finalHeading
            );

            return this.goToPoseFunction(
                    desiredPose, distanceThreshold, angleThreshold,
                    useStoppingDistance, packet
            );
        }

        packet.addLine("follower: pure pursuit");
        if (useStoppingDistance) {
            pose = pose.plus(this.stoppingDistance(packet));
            packet.addLine("follower: stop dist. used");
        }

        stoppingDistancePose = pose;

        SimpleMatrix desiredPose = this.geometricController.calculate(pose, path);
        SimpleMatrix wheelSpeeds
                = mecanumKinematicModel.inverseKinematics(followController.calculate(
                pose,
                desiredPose
        ));

        double maxScale = (maxSpeed / MECHANICAL_PARAMETERS.wheelRadius)
                / wheelSpeeds.elementMaxAbs();

        wheelSpeeds = wheelSpeeds.scale(maxScale);

        SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);

        setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);

        double[] desiredPosition = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        Drawing.drawPoint(desiredPosition, canvas, "green");

        return true;
    }

    public Action followPath(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance
    ) {
        Drivetrain drivetrain = this;
        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drivetrain.localize(packet);

                return drivetrain.followPathFunction(
                        path,
                        maxSpeed,
                        distanceThreshold,
                        angleThreshold,
                        useStoppingDistance,
                        packet
                );
            }
        };

    }


    /**
     * Stops the Motors of the drivetrain immediately.
     *
     * @return An InstantAction that sets all wheel powers to zero.
     */
    public InstantAction stopMotors() {
        return new InstantAction(() -> setPower(stopMatrix));
    }


    public void updateTelemetry(TelemetryPacket packet) {
        if (!DebuggingParameters.debug) return;

        packet.put("x position (in)", state.get(0, 0));
        packet.put("y position (in)", state.get(1, 0));
        packet.put("heading (deg)", Math.toDegrees(state.get(2, 0)));
        packet.put("longitudinal Velocity (in/s)", state.get(3, 0));
        packet.put("lateral Velocity (in/s)", state.get(4, 0));
        packet.put("heading Velocity (deg/s)", Math.toDegrees(state.get(5, 0)));


        Canvas canvas = packet.fieldOverlay();
        Drawing.drawRobot(state, canvas, "black");
    }


    /**
     * Allows for manual control of Robot using controller joystick.
     *
     * @param ly Left stick Y axis (forward/backward)
     * @param lx Left stick X axis (strafe left/right)
     * @param rX Right stick X axis (rotation)
     * @return An Action that applies the joystick values to the drivetrain for manual driving.
     */
    public Action manualControl(double ly, double lx, double rX) {
        Drivetrain drivetrain = this;

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drivetrain.localize(packet);

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
                return false;
            }
        };
    }

    public static class MotionParameters {
        public double maxSpeed = 110.0; // (in/s)
    }

    public static class PoseConstants {
        public PIDConstants xPIDConstants = new PIDConstants(4.5, 0, 0);

        public PIDConstants yPIDConstants = new PIDConstants(4.5, 0, 0);

        public PIDConstants headingPIDConstants = new PIDConstants(1.8, 0, 0);
    }

    public static class PoseConstantsGeo {
        public PIDConstants xPIDConstants = new PIDConstants(7, 0, 0);

        public PIDConstants yPIDConstants = new PIDConstants(7, 0, 0);

        public PIDConstants headingPIDConstants = new PIDConstants(7, 0, 0);
    }

    public static class FFConstantsController {
        public FFConstants lf = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants lb = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants rb = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants rf = new FFConstants(0.0105, 0.124, 0.0165);
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
        public static double distanceThreshold = 1;
        /**
         * The acceptable difference between wanted and current angles (Radians) Used to save time &
         * reduce unnecessary movements
         */
        public static double angleThreshold = 0.05;
        public static boolean stopPlanning = true;
    }
}
