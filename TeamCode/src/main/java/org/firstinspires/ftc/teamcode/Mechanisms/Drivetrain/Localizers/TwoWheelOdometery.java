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

package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
public class TwoWheelOdometery {

    public static double xOffset = -139.7; //MM
    public static double yOffset = -50.8; //MM
    GoBildaPinpointDriver odo;
    HardwareMap hardwareMap;

    public TwoWheelOdometery(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        this.odo.resetPosAndIMU();
    }

    public SimpleMatrix calculate() {
        odo.update();
        SimpleMatrix globalRelativeTVelocities = new SimpleMatrix(
                new double[][]{
                        new double[]{odo.getVelX(DistanceUnit.INCH)},
                        new double[]{odo.getVelY(DistanceUnit.INCH)},
                        new double[]{odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)}
                }
        );

        SimpleMatrix robotRelativeVelocities = Utils.rotateGlobalToBody(globalRelativeTVelocities, odo.getHeading(AngleUnit.RADIANS));

        return new SimpleMatrix(
                new double[][]{
                        new double[]{odo.getPosition().getX(DistanceUnit.INCH)},
                        new double[]{odo.getPosition().getY(DistanceUnit.INCH)},
                        new double[]{odo.getHeading(AngleUnit.RADIANS)},
                        new double[]{robotRelativeVelocities.get(0, 0)},
                        new double[]{robotRelativeVelocities.get(1, 0)},
                        new double[]{robotRelativeVelocities.get(2, 0)},

                }
        );
    }

    public void resetPosAndRecalibrateIMU() {
        odo.resetPosAndIMU();
    }

    public void reCalibrateIMU() {
        odo.recalibrateIMU();
    }
}
