package org.firstinspires.ftc.teamcode.sim;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.control.StoppingDistance;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumKinematicModel;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * A plant model for the drivetrain, so the control maths can be driven closed-loop off the robot.
 * <p>
 * This is not a physics simulation. It models exactly the parts that decide whether the
 * controllers behave: the wheel model, the power ceiling the feedforward runs into, and rigid-body
 * integration of the resulting twist. Velocity follows the command with a first order lag rather
 * than instantly, which is what makes coasting and overshoot show up at all.
 * <p>
 * Everything it uses -- {@link MecanumKinematicModel}, {@link Utils}, {@link StoppingDistance} and
 * the {@link Drivetrain} parameter blocks -- is the real production code, so a test written
 * against this exercises the shipped maths rather than a copy of it.
 */
public class DrivetrainSim {

    /** Fastest a wheel can turn once the feedforward saturates at full power. (rad/s) */
    public static final double MAX_WHEEL_SPEED =
            (1.0 - Drivetrain.FF_CONSTANTS.lf.kS) / Drivetrain.FF_CONSTANTS.lf.kV;

    private static final double VELOCITY_LAG = 0.25; // (s) time constant of the velocity response

    private final MecanumKinematicModel kinematics =
            new MecanumKinematicModel(Drivetrain.MECHANICAL_PARAMETERS);

    /** x (in), y (in), heading (rad). */
    private SimpleMatrix pose = new SimpleMatrix(3, 1);
    /** Body frame vx (in/s), vy (in/s), omega (rad/s). */
    private SimpleMatrix velocity = new SimpleMatrix(3, 1);

    public DrivetrainSim(double x, double y, double headingRad) {
        pose.set(0, 0, x);
        pose.set(1, 0, y);
        pose.set(2, 0, headingRad);
    }

    public SimpleMatrix pose() {
        return pose.copy();
    }

    public SimpleMatrix velocity() {
        return velocity.copy();
    }

    public double x() {
        return pose.get(0, 0);
    }

    public double y() {
        return pose.get(1, 0);
    }

    public double headingRad() {
        return pose.get(2, 0);
    }

    public double speed() {
        return Math.hypot(velocity.get(0, 0), velocity.get(1, 0));
    }

    /**
     * The pose the robot would coast to if power were cut now, in the field frame.
     * <p>
     * The same calculation Drivetrain.computeStoppingDistance performs, against the same
     * coefficients.
     */
    public SimpleMatrix driftedPose() {
        Drivetrain.StoppingDistanceParameters c = Drivetrain.STOPPING_DISTANCE_PARAMETERS;
        SimpleMatrix drift = new SimpleMatrix(new double[][]{
                {StoppingDistance.forVelocity(velocity.get(0, 0), c.xLinear, c.xQuadratic)},
                {StoppingDistance.forVelocity(velocity.get(1, 0), c.yLinear, c.yQuadratic)},
                {StoppingDistance.forVelocity(velocity.get(2, 0), c.headingLinear,
                        c.headingQuadratic)}
        });
        return pose.plus(Utils.rotateBodyToGlobal(drift, pose.get(2, 0)));
    }

    /**
     * Applies a commanded body twist for one tick.
     *
     * @param twist 3x1 of vx (in/s), vy (in/s), omega (rad/s), as the controllers produce it
     * @param dt tick length (s)
     */
    public void step(SimpleMatrix twist, double dt) {
        SimpleMatrix achievable = capToWheelCeiling(twist);

        // First order lag towards the commanded twist.
        double alpha = Math.min(1.0, dt / VELOCITY_LAG);
        velocity = velocity.plus(achievable.minus(velocity).scale(alpha));

        integrate(dt);
    }

    /** Cuts power: the robot coasts to a stop rather than stopping dead. */
    public void coast(double dt) {
        double alpha = Math.min(1.0, dt / VELOCITY_LAG);
        velocity = velocity.scale(1.0 - alpha);
        integrate(dt);
    }

    /**
     * Scales a twist down until no wheel is asked to exceed what full power can deliver.
     * <p>
     * Scaling the whole vector is what Drivetrain.setPower does when it normalises, so the
     * direction of travel is preserved and only the magnitude gives.
     */
    public SimpleMatrix capToWheelCeiling(SimpleMatrix twist) {
        SimpleMatrix wheelSpeeds = kinematics.inverseKinematics(twist);
        double fastest = wheelSpeeds.elementMaxAbs();
        if (fastest <= MAX_WHEEL_SPEED || fastest == 0) {
            return twist;
        }
        return twist.scale(MAX_WHEEL_SPEED / fastest);
    }

    private void integrate(double dt) {
        SimpleMatrix fieldVelocity = Utils.rotateBodyToGlobal(velocity, pose.get(2, 0));
        pose = pose.plus(fieldVelocity.scale(dt));
        pose.set(2, 0, Utils.angleWrap(pose.get(2, 0)));
    }
}
