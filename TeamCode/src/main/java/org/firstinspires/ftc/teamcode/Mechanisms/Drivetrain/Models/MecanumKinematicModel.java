package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Models;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

public class MecanumKinematicModel {

    /**
     * <p>4×3 inverse-kinematics Jacobian matrix for a mecanum-drive robot.</p>
     *
     * <p>Maps a chassis twist <code>[vₓ, vᵧ, ω]</code> in the robot frame to raw wheel speeds
     * before scaling by <code>(1 / wheelRadius)</code>.  The rows correspond to:</p>
     * <pre>
     * [ωₗf, ωₗb, ωᵣb, ωᵣf]ᵀ = (1 / r) · Hᵢ · [vₓ, vᵧ, ω]ᵀ
     * </pre>
     *
     * <p>Hᵢ =</p>
     * <pre>
     * [  1, –1, –(ℓ + b);
     *    1,  1, –(ℓ + b);
     *    1, –1,  (ℓ + b);
     *    1,  1,  (ℓ + b) ]
     * </pre>
     *
     * <p>where:</p>
     * <ul>
     *   <li><code>ℓ</code> = <code>longDistToAxles</code> (half the wheelbase, in meters)</li>
     *   <li><code>b</code> = <code>latDistToAxles</code> (half the track width, in meters)</li>
     * </ul>
     *
     * <p>Both <code>longDistToAxles</code> and <code>latDistToAxles</code> must be initialized
     * before this matrix is constructed.  Used by {@link #inverseKinematics(SimpleMatrix)} to
     * convert chassis velocities into individual wheel angular velocities.</p>
     */
    static SimpleMatrix inverseJacobian = new SimpleMatrix(
            new double[][]{
                    new double[]{
                            1d,
                            -1d,
                            -(Drivetrain.MechanicalParameters.longDistToAxles
                                    + Drivetrain.MechanicalParameters.latDistToAxles)
                    },
                    new double[]{
                            1d,
                            1d,
                            -(Drivetrain.MechanicalParameters.longDistToAxles
                                    + Drivetrain.MechanicalParameters.latDistToAxles)
                    },
                    new double[]{
                            1d,
                            -1d,
                            (Drivetrain.MechanicalParameters.longDistToAxles
                                    + Drivetrain.MechanicalParameters.latDistToAxles)
                    },
                    new double[]{
                            1d,
                            1d,
                            (Drivetrain.MechanicalParameters.longDistToAxles
                                    + Drivetrain.MechanicalParameters.latDistToAxles)
                    }
            }
    );
    Drivetrain.MechanicalParameters mechanicalParameters;

    public MecanumKinematicModel(Drivetrain.MechanicalParameters mechanicalParameters) {
        this.mechanicalParameters = mechanicalParameters;
    }

    /**
     * Computes the individual wheel angular velocities required to achieve a desired chassis
     * twist.
     * <p>
     * Uses the inverse kinematics relationship:
     * <pre>
     *     u = (1/r) · Hₖᵢ · ξ
     * </pre>
     * where
     * <ul>
     *   <li>u = [ω<sub>lf</sub>, ω<sub>lb</sub>, ω<sub>rb</sub>, ω<sub>rf</sub>]<sup>T</sup> are
     *   the wheel speeds (rad/s)</li>
     *   <li>ξ = [v<sub>x</sub>, v<sub>y</sub>, ω]<sup>T</sup> is the robot twist in the body
     *   frame (m/s, m/s, rad/s)</li>
     *   <li>r is the wheel radius (m)</li>
     *   <li>Hₖᵢ is the 4×3 inverse-kinematic matrix for a mecanum drive:
     *       <pre>
     *       [ 1, –1, –(ℓ+b),
     *         1,  1, –(ℓ+b),
     *         1, –1,  (ℓ+b),
     *         1,  1,  (ℓ+b) ]
     *       </pre>
     *     where 2ℓ is track-width and 2b is wheelbase.  [oai_citation:0‡models-and-transforms-1
     *     .pdf](file-service://file-7rMJpS2Xmm7MWCGzKFFvQ2)
     *     [oai_citation:1‡models-and-transforms-1.pdf](file-service://file-7rMJpS2Xmm7MWCGzKFFvQ2)
     * </ul>
     *
     * @param twist a 3×1 SimpleMatrix representing [v<sub>x</sub>; v<sub>y</sub>; ω] in the robot
     * (body) frame; units are (m/s, m/s, rad/s)
     *
     * @return a 4×1 SimpleMatrix of wheel angular velocities [ω<sub>lf</sub>; ω<sub>lb</sub>;
     * ω<sub>rb</sub>; ω<sub>rf</sub>] in rad/s
     *
     * @throws IllegalStateException if {@code inverseJacobian} or {@code wheelRadius} has not been
     * initialized
     */
    public SimpleMatrix inverseKinematics(SimpleMatrix twist) {
        SimpleMatrix wheelSpeeds =
                inverseJacobian.scale(1 / mechanicalParameters.wheelRadius).mult(twist);
        return wheelSpeeds;
    }
}
