package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import org.firstinspires.ftc.teamcode.util.Timed;

import static org.firstinspires.ftc.teamcode.util.Utils.makePoseVector;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.FFConstants;
import org.firstinspires.ftc.teamcode.control.PIDConstants;
import org.firstinspires.ftc.teamcode.control.StoppingDistance;
import org.firstinspires.ftc.teamcode.hardware.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.util.Utils;

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
    public static PoseConstants POSE_CONSTANTS = new PoseConstants();
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants();
    public static FFConstantsController FF_CONSTANTS = new FFConstantsController();
    public static MechanicalParameters MECHANICAL_PARAMETERS = new MechanicalParameters();
    public static ThresholdParameters THRESHOLD_PARAMETERS = new ThresholdParameters();
    public static DebuggingParameters DEBUGGING_PARAMETERS = new DebuggingParameters();
    public static StoppingDistanceParameters STOPPING_DISTANCE_PARAMETERS =
            new StoppingDistanceParameters();
    private static Drivetrain instance;
    public final TwoWheelOdometery twoWheelOdo;
    public final DcMotorAdvanced motorLeftFront;
    public final DcMotorAdvanced motorLeftBack;
    public final DcMotorAdvanced motorRightBack;
    public final DcMotorAdvanced motorRightFront;
    private final DrivetrainMotorController motorController;
    private final GeometricController geometricController;
    private final SimpleMatrix stopMatrix = new SimpleMatrix(4, 1);
    private final PoseController poseController;
    private final PoseController followController;
    private final MecanumKinematicModel mecanumKinematicModel;
    public SimpleMatrix shootWhileMovingPose;
    public SimpleMatrix driftedPose;
    public SimpleMatrix state;
    private TelemetryPacket packet;

    /**
     * Initializes the Drivetrain (Wheels of the Robot)
     *
     * @param hardwareMap The hardwareMap of the Robot, describes which port of the hub is connected
     * to which name
     */
    private Drivetrain(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.motorController = new DrivetrainMotorController(FF_CONSTANTS);
        this.poseController = new PoseController(
                POSE_CONSTANTS.xPIDConstants,
                POSE_CONSTANTS.yPIDConstants,
                POSE_CONSTANTS.headingPIDConstants
        );
        this.geometricController = new GeometricController(
                FOLLOWER_CONSTANTS.positionLookahead,
                FOLLOWER_CONSTANTS.headingLookahead
        );
        this.followController = new PoseController(
                FOLLOWER_CONSTANTS.xPIDConstants,
                FOLLOWER_CONSTANTS.yPIDConstants,
                FOLLOWER_CONSTANTS.headingPIDConstants
        );

        this.twoWheelOdo = new TwoWheelOdometery(hardwareMap);

        this.mecanumKinematicModel = new MecanumKinematicModel(MECHANICAL_PARAMETERS);

        this.motorLeftFront = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.LEFT_FRONT_MOTOR),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );
        this.motorLeftBack = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.LEFT_BACK_MOTOR),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );
        this.motorRightBack = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.RIGHT_BACK_MOTOR),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );
        this.motorRightFront = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.RIGHT_FRONT_MOTOR),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );

        this.motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

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

        this.state = new SimpleMatrix(6, 1);
        this.driftedPose = new SimpleMatrix(3, 1);
        this.shootWhileMovingPose = new SimpleMatrix(3, 1);

        this.twoWheelOdo.resetPosAndRecalibrateIMU();
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Drivetrain(hardwareMap);
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Drivetrain not initialized!");
        }
        return instance;
    }

    /**
     * Sets the Position of the bot in its start position.
     * <p>
     * NOTE: Make sure 'setTelemetry' is called before this.
     *
     * @param xPosition Initial X position (inches)
     * @param yPosition Initial Y position (inches)
     * @param heading Initial heading (degrees)
     */
    public void setInitialPose(double xPosition, double yPosition, double heading) {
        this.twoWheelOdo.odo.setPosX(xPosition, DistanceUnit.INCH);
        this.twoWheelOdo.odo.setPosY(yPosition, DistanceUnit.INCH);
        this.twoWheelOdo.odo.setHeading(heading, AngleUnit.DEGREES);

        this.localize();
        this.updateTelemetry();
    }

    public void localize() {
        this.state = this.twoWheelOdo.calculate();

        this.driftedPose =
                this.state.extractMatrix(0, 3, 0, 1).plus(this.computeStoppingDistance());

        // state rows: 0 x (in), 1 y (in), 2 heading (rad),
        //             3 long. vel (in/s), 4 lat. vel (in/s), 5 turn rate (rad/s)
        SimpleMatrix bodyFrameVelocities = state.extractMatrix(3, 6, 0, 1);
        SimpleMatrix fieldFrameVelocites = Utils.rotateBodyToGlobal(
                bodyFrameVelocities,
                this.state.get(2, 0)
        );

        SimpleMatrix shootWhileMovingPose = new SimpleMatrix(3, 1);
        shootWhileMovingPose.set(
                0,
                0,
                this.state.get(0, 0)
                        + THRESHOLD_PARAMETERS.compensationFactor * fieldFrameVelocites.get(0, 0)

        );
        shootWhileMovingPose.set(
                1,
                0,
                this.state.get(1, 0)
                        + THRESHOLD_PARAMETERS.compensationFactor * fieldFrameVelocites.get(1, 0)

        );
        shootWhileMovingPose.set(
                2,
                0,
                this.state.get(2, 0)
                        + THRESHOLD_PARAMETERS.compensationFactorTheta * fieldFrameVelocites.get(
                        2,
                        0
                )

        );
        this.shootWhileMovingPose = shootWhileMovingPose;
    }

    private void updateTelemetry() {
        Canvas canvas = packet.fieldOverlay();
        Drawing.drawRobot(state, canvas, "black");
        if (!DEBUGGING_PARAMETERS.printTelemetry) return;

        this.packet.put("x pos (in)", this.state.get(0, 0));
        this.packet.put("y pos (in)", this.state.get(1, 0));
        this.packet.put("heading (deg)", Math.toDegrees(this.state.get(2, 0)));
        this.packet.put("long. vel (in/s)", this.state.get(3, 0));
        this.packet.put("lat vel (in/s)", this.state.get(4, 0));
        this.packet.put("yaw vel (deg/s)", Math.toDegrees(this.state.get(5, 0)));
    }

    /**
     * How far the robot coasts from its current body-frame velocity, in the field frame.
     *
     * @return 3x1 of x (in), y (in), heading (rad) drift
     */
    private SimpleMatrix computeStoppingDistance() {
        StoppingDistanceParameters coeffs = STOPPING_DISTANCE_PARAMETERS;
        SimpleMatrix stopDistance = new SimpleMatrix(
                new double[][]{
                        new double[]{
                                StoppingDistance.forVelocity(
                                        this.state.get(3, 0),
                                        coeffs.xLinear,
                                        coeffs.xQuadratic
                                )
                        },
                        new double[]{
                                StoppingDistance.forVelocity(
                                        this.state.get(4, 0),
                                        coeffs.yLinear,
                                        coeffs.yQuadratic
                                )
                        },
                        new double[]{
                                StoppingDistance.forVelocity(
                                        this.state.get(5, 0),
                                        coeffs.headingLinear,
                                        coeffs.headingQuadratic
                                )
                        }
                }
        );

        return Utils.rotateBodyToGlobal(stopDistance, this.state.get(2, 0));
    }

    /**
     * Powers the four motors (lfm, lbm, rbm, rfm). Scales the vector down if any wheel exceeds
     * full power, preserving the ratios between them. A non-finite command stops the drivetrain
     * rather than reaching the motors.
     *
     * @param powers matrix of wheel power values (order: lfm, lbm, rbm, rfm)
     */
    public void setPower(SimpleMatrix powers) {
        for (int i = 0; i < 4; i++) {
            if (!Double.isFinite(powers.get(i, 0))) {
                powers = stopMatrix;
                break;
            }
        }

        double largest = powers.elementMaxAbs();
        if (largest > 1.0) {
            powers = powers.scale(1.0 / largest);
        }
        double powerLeftFront = powers.get(0, 0);
        double powerLeftBack = powers.get(1, 0);
        double powerRightBack = powers.get(2, 0);
        double powerRightFront = powers.get(3, 0);

        //        this.packet.put("front-left pow.", powerLeftFront);
        //        this.packet.put("front-right pow.", powerRightFront);
        //        this.packet.put("back-left pow.", powerLeftBack);
        //        this.packet.put("back-right pow.", powerRightBack);

        motorLeftFront.setPower(powerLeftFront);
        motorLeftBack.setPower(powerLeftBack);
        motorRightBack.setPower(powerRightBack);
        motorRightFront.setPower(powerRightFront);

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
        // FF_CONSTANTS are fitted against wheel rad/s, so no gear reduction is applied here.
        setPower(motorController.calculate(wheelSpeeds, wheelAccelerations));
    }

    public boolean inPositionThreshold(
            double[] desiredPos, double distanceThreshold
    ) {
        double distanceToGoal = Utils.calculateDistance(
                this.state.get(0, 0),
                this.state.get(1, 0),
                desiredPos[0],
                desiredPos[1]
        );
        //        this.packet.put("dist. to goal/final point (in)", distanceToGoal);
        //        this.packet.put("dist. thresh (in)", distanceThreshold);
        return Math.abs(distanceToGoal) <= Math.abs(distanceThreshold);
    }

    private boolean inHeadingThreshold(double heading, double angleThreshold) {
        double headingError = Utils.angleWrap(heading - this.state.get(2, 0));
        //        this.packet.put("heading error (deg)", Math.toDegrees(headingError));
        //        this.packet.put("angle thresh (deg)", Math.toDegrees(angleThreshold));
        return Math.abs(headingError) <= Math.abs(angleThreshold);
    }

    public boolean inStoppingZone(
            SimpleMatrix desiredPose, double distanceThreshold,
            double angleThreshold
    ) {
        double[] desiredPosition = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        return inPositionThreshold(desiredPosition, distanceThreshold) && this.inHeadingThreshold(
                desiredPose.get(2, 0),
                angleThreshold
        );
    }

    private boolean goToPoseFunction(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance
    ) {
        boolean readyToStop = this.inStoppingZone(
                desiredPose,
                distanceThreshold,
                angleThreshold
        );
        if (readyToStop) {
            setPower(this.stopMatrix);
            //            this.packet.addLine("pose control: robot within thresh.");
            return false;
        }

        Canvas canvas = this.packet.fieldOverlay();

        double[] desiredPosition = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        //        this.packet.put("target x pos (in)", desiredPose.get(0, 0));
        //        this.packet.put("target y pos (in)", desiredPose.get(1, 0));
        //        this.packet.put("target heading (deg)", Math.toDegrees(desiredPose.get(2, 0)));
        Drawing.drawCircle(desiredPosition, canvas, distanceThreshold, "green", false);

        SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);
        if (useStoppingDistance) {
            pose = this.driftedPose;
            //            this.packet.addLine("pose control: drift used");
            Drawing.drawRobot(this.driftedPose, canvas, "blue");
        }

        SimpleMatrix targetTwist = this.poseController.calculate(pose, desiredPose);
        //        packet.put("target long. vel. (in/s)", targetTwist.get(0, 0));
        //        packet.put("target lat. vel. (in/s)", targetTwist.get(1, 0));
        //        packet.put("target yaw rate (deg/s)", Math.toDegrees(targetTwist.get(2, 0)));

        SimpleMatrix wheelSpeeds = this.mecanumKinematicModel.inverseKinematics(targetTwist);

        //        packet.put("target long. vel. (in/s)", targetTwist.get(0, 0));
        //        packet.put("target lat. vel. (in/s)", targetTwist.get(1, 0));
        //        packet.put("target yaw rate (deg/s)", Math.toDegrees(targetTwist.get(2, 0)));

        SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);

        this.setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);

        return true;
    }

    public Action goToPose(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance
    ) {
        Drivetrain drivetrain = this;
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    drivetrain.poseController.reset();
                    started = true;
                }

                drivetrain.setTelemetry(packet);
                drivetrain.localize();
                drivetrain.updateTelemetry();

                return drivetrain.goToPoseFunction(
                        desiredPose, distanceThreshold,
                        angleThreshold,
                        useStoppingDistance
                );
            }
        };

    }

    /** goToPose, giving up after seconds (s) and cutting power. */
    public Action goToPoseTimed(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance, double seconds
    ) {
        return Timed.deadline(
                goToPose(desiredPose, distanceThreshold, angleThreshold, useStoppingDistance),
                seconds,
                () -> setPower(stopMatrix)
        );
    }

    private boolean followPathFunction(
            Path path, double maxSpeed, double distanceThreshold, double angleThreshold
            , boolean useStoppingDistance
    ) {
        Canvas canvas = this.packet.fieldOverlay();

        Drawing.drawPath(path.getWaypoints(), canvas, "red");

        if (this.inPositionThreshold(
                path.getFinalPoint(),
                FOLLOWER_CONSTANTS.poseControlHandoffDistance
        )) {
            //            packet.addLine("follower: pose control");

            SimpleMatrix desiredPose = makePoseVector(
                    path.getFinalPoint()[0], path.getFinalPoint()[1],
                    Math.toDegrees(path.finalHeading)
            );

            boolean isRunning = this.goToPoseFunction(
                    desiredPose, distanceThreshold, angleThreshold,
                    useStoppingDistance
            );

            return isRunning;
        }

        double[] position = {state.get(0, 0), state.get(1, 0)};
        SimpleMatrix pose = state.extractMatrix(0, 3, 0, 1);

        Drawing.drawCircle(position, canvas, FOLLOWER_CONSTANTS.positionLookahead, "purple", false);
        Drawing.drawCircle(position, canvas, FOLLOWER_CONSTANTS.headingLookahead, "orange", false);

        //        packet.addLine("follower: pure pursuit");
        if (useStoppingDistance) {
            pose = this.driftedPose;
            //            packet.addLine("follower: drift used");
            Drawing.drawRobot(this.driftedPose, canvas, "blue");
        }

        // TODO: try with drift pose too
        SimpleMatrix desiredPose = this.geometricController.calculate(
                state.extractMatrix(
                        0,
                        3,
                        0,
                        1
                ), path
        );
        SimpleMatrix wheelSpeeds
                = mecanumKinematicModel.inverseKinematics(followController.calculate(
                pose,
                desiredPose
        ));

        // Cap the wheel speeds at maxSpeed without scaling a small correction up to it. Scaling
        // the whole vector keeps the ratios, so the direction of travel is unchanged either way.
        double fastestWheel = wheelSpeeds.elementMaxAbs();
        double speedLimit = maxSpeed / MECHANICAL_PARAMETERS.wheelRadius;
        if (fastestWheel > speedLimit) {
            wheelSpeeds = wheelSpeeds.scale(speedLimit / fastestWheel);
        }

        SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);

        setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);

        double[] desiredPosition = {desiredPose.get(0, 0), desiredPose.get(1, 0)};
        Drawing.drawPoint(desiredPosition, canvas, "green");

        return true;
    }

    /**
     * @param path
     * @param maxSpeed ~120
     * @param distanceThreshold INCHES
     * @param angleThreshold RADIANS
     * @param useStoppingDistance
     *
     * @return
     */
    public Action followPath(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance
    ) {
        Drivetrain drivetrain = this;
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // The lookahead index only moves forward, so it has to be rewound for each new
                // path. Doing it here rather than at the end of the previous path covers the
                // paths that stop on a deadline or never reach their threshold.
                if (!started) {
                    drivetrain.geometricController.resetLookAhead();
                    drivetrain.followController.reset();
                    drivetrain.poseController.reset();
                    started = true;
                }

                drivetrain.setTelemetry(packet);
                drivetrain.localize();
                drivetrain.updateTelemetry();

                return drivetrain.followPathFunction(
                        path,
                        maxSpeed,
                        distanceThreshold,
                        angleThreshold,
                        useStoppingDistance
                );
            }
        };

    }

    /**
     * @param path
     * @param maxSpeed ~120
     * @param distanceThreshold INCHES
     * @param angleThreshold RADIANS
     * @param useStoppingDistance
     *
     * @return
     */
    /** followPath, giving up after seconds (s) and cutting power. */
    public Action followPathTimed(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            boolean useStoppingDistance,
            double seconds
    ) {
        return Timed.deadline(
                followPath(path, maxSpeed, distanceThreshold, angleThreshold,
                        useStoppingDistance),
                seconds,
                () -> setPower(stopMatrix)
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
     * Allows for manual control of Robot using controller joystick.
     * <p>
     * All three are raw stick axes, which point the opposite way to the robot frame, so each is
     * negated on the way in. Callers pass (-left_stick_x, left_stick_y, right_stick_x).
     *
     * @param negatedStrafe negated left stick X axis; positive strafes left
     * @param forwardAxis left stick Y axis; negative is forward on a gamepad
     * @param turnAxis right stick X axis; positive turns clockwise
     *
     * @return An Action that applies the joystick values to the drivetrain for manual driving.
     */
    public Action manualControl(double negatedStrafe, double forwardAxis, double turnAxis) {
        Drivetrain drivetrain = this;

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drivetrain.setTelemetry(packet);
                drivetrain.localize();
                drivetrain.updateTelemetry();

                double x = -forwardAxis;   // longitudinal, positive forward
                double y = negatedStrafe;  // lateral, positive left
                double rx = -turnAxis;     // yaw, positive counter-clockwise
                SimpleMatrix compensatedTwist = new SimpleMatrix(
                        new double[][]{
                                new double[]{MECHANICAL_PARAMETERS.wheelRadius * x},
                                new double[]{MECHANICAL_PARAMETERS.wheelRadius * y},
                                new double[]{
                                        (MECHANICAL_PARAMETERS.wheelRadius / (
                                                MECHANICAL_PARAMETERS.longDistToAxles
                                                        + MECHANICAL_PARAMETERS.latDistToAxles))
                                                * rx
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

    public void setTelemetry(TelemetryPacket packet) {
        this.packet = packet;
        this.geometricController.setTelemetry(packet);
    }

    public static class PoseConstants {
        public PIDConstants xPIDConstants = new PIDConstants(4.5, 0, 0);
        public PIDConstants yPIDConstants = new PIDConstants(4.5, 0, 0);
        public PIDConstants headingPIDConstants = new PIDConstants(1.8, 0, 0);
    }

    public static class FollowerConstants {
        public PIDConstants xPIDConstants = new PIDConstants(4.5, 0, 0);
        public PIDConstants yPIDConstants = new PIDConstants(4.5, 0, 0);
        public PIDConstants headingPIDConstants = new PIDConstants(1.8, 0, 0);
        public double positionLookahead = 20.0; // (in)
        public double headingLookahead = 30.0; // (in)
        /** Distance from the last waypoint at which the follower hands off to pose control. (in) */
        public double poseControlHandoffDistance = 30.0;
    }

    public static class FFConstantsController {
        public FFConstants lf = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants lb = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants rb = new FFConstants(0.0105, 0.124, 0.0165);
        public FFConstants rf = new FFConstants(0.0105, 0.124, 0.0165);
    }

    public static class MechanicalParameters {
        public double wheelRadius = 2.16535; // (in)
        public double longDistToAxles = 5.7; // (in) Longitudinal distance from center to axles
        public double latDistToAxles = 5.31496; // (in) Lateral distance from center to axles
    }

    /**
     * Debugging parameters for the Drivetrain. Can turn telemetry printouts on/off globally.
     */
    public static class DebuggingParameters {
        /**
         * Set to true to enable telemetry printouts, false to disable
         */
        public boolean printTelemetry = true;
    }

    /** Coast fits measured with TuneAccel. Body frame; heading terms are in radians. */
    public static class StoppingDistanceParameters {
        public double xLinear = 0.132;        // (in per in/s)
        public double xQuadratic = 0.00132;   // (in per (in/s)^2)
        public double yLinear = 0.0716;       // (in per in/s)
        public double yQuadratic = 0.00213;   // (in per (in/s)^2)
        public double headingLinear = 0.0658;      // (rad per rad/s)
        public double headingQuadratic = 0.00522;  // (rad per (rad/s)^2)
    }

    public static class ThresholdParameters {
        public double maxVoltage = 12.5; // (V)
        public double acceptablePowerDifference = 0.0001;
        public double distanceThreshold = 1.0;
        public double angleThreshold = Math.toRadians(2.5);
        public boolean useStoppingDistance = true;
        public double compensationFactor = 0;
        public double compensationFactorTheta = 0.12;
    }
}
