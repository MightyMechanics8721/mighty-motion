/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class TwoWheelOdometry {

    public static double xOffset = 155; // (mm) forward pod from centre of rotation
    public static double yOffset = 43.13; // (mm) lateral pod from centre of rotation

    /**
     * Whether the Pinpoint reports velocity in the field frame, the same frame as its position.
     * <p>
     * The driver documents getVelX/getVelY only as "X (forward)" and "Y (strafe)", which names the
     * pods rather than the frame, so this cannot be settled from the SDK. Run the "Tune 2 Wheel
     * Localizer" OpMode to settle it: turn the robot to 90 degrees and push it straight forward.
     * With this set correctly, long. vel is positive and lat. vel is near zero. If they come out
     * swapped, flip this on the dashboard.
     * <p>
     * It matters because driftedPose feeds every autonomous path, and a wrong frame corrupts the
     * drift at every heading except zero.
     */
    public static boolean velocityIsFieldFrame = true;
    public GoBildaPinpointDriver odo;
    HardwareMap hardwareMap;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, HardwareNames.ODOMETRY);
        this.odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        this.odo.resetPosAndIMU();
    }

    /**
     * @return 6x1 state: x (in), y (in), heading (rad), then body-frame vx (in/s), vy (in/s),
     * omega (rad/s)
     * <p>
     * Velocities are resolved into the body frame according to velocityIsFieldFrame; see that
     * field for why the frame is a setting rather than a fact, and how to confirm it.
     */
    public SimpleMatrix calculate() {
        odo.update();

        SimpleMatrix robotRelativeVelocities = toBodyFrame(reportedVelocities());

        return new SimpleMatrix(
                new double[][]{
                        new double[]{odo.getPosition().getX(DistanceUnit.INCH)},
                        new double[]{odo.getPosition().getY(DistanceUnit.INCH)},
                        new double[]{odo.getHeading(UnnormalizedAngleUnit.RADIANS)},
                        new double[]{robotRelativeVelocities.get(0, 0)},
                        new double[]{robotRelativeVelocities.get(1, 0)},
                        new double[]{robotRelativeVelocities.get(2, 0)},
                        }
        );
    }

    /**
     * @return 3x1 of the velocities exactly as the Pinpoint reports them: x (in/s), y (in/s),
     * heading (rad/s), in whichever frame the device uses
     */
    public SimpleMatrix reportedVelocities() {
        return new SimpleMatrix(
                new double[][]{
                        new double[]{odo.getVelX(DistanceUnit.INCH)},
                        new double[]{odo.getVelY(DistanceUnit.INCH)},
                        new double[]{odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)}
                }
        );
    }

    /** Resolves reported velocities into the body frame, per velocityIsFieldFrame. */
    public SimpleMatrix toBodyFrame(SimpleMatrix reported) {
        if (!velocityIsFieldFrame) {
            return reported;
        }
        return Utils.rotateGlobalToBody(reported, odo.getHeading(AngleUnit.RADIANS));
    }

    public void resetPosAndRecalibrateIMU() {
        odo.resetPosAndIMU();
    }

    public void reCalibrateIMU() {
        odo.recalibrateIMU();
    }
}
