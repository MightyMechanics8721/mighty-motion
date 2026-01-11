package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Utility class to easily get the horizontal angle (tx)
 * from the Limelight to the detected AprilTag.
 */
public class AprilTagUtils {

    private final Limelight3A limelight;


    /**
     * Initializes the utility with the Limelight instance.
     *
     * @param hardwareMap the FTC hardware map
     * @param limelightName the configured name of your Limelight in the RC config
     */
    public AprilTagUtils(HardwareMap hardwareMap, String limelightName) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // switch to AprilTag pipeline
        limelight.start();

    }

    /**
     * Returns the horizontal angle (tx) to the first detected AprilTag in degrees.
     *
     * @return horizontal angle in degrees, or Double.NaN if no tag is detected
     */
    public double getHorizontalAngle() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            FiducialResult fiducial = result.getFiducialResults().get(0);
            return fiducial.getTargetXDegrees();
        }
        return Double.NaN; // indicates no valid target
    }

    public double getPos() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            FiducialResult fiducial = result.getFiducialResults().get(0);
            return fiducial.getTargetXDegrees();
        }
        return Double.NaN; // indicates no valid target
    }






    /**
     * Checks whether a valid AprilTag is currently detected.
     *
     * @return true if a tag is detected, false otherwise
     */
    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && !result.getFiducialResults().isEmpty();
    }
}
