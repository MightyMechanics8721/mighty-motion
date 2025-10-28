package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class to easily get AprilTag IDs using EasyOpenCV (VisionPortal + Webcam).
 */
public class AprilTagIDUtils {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    /**
     * Initializes AprilTag detection with VisionPortal.
     *
     * @param hardwareMap the FTC hardware map
     * @param webcamName the configured name of your webcam in the RC configuration
     */
    public AprilTagIDUtils(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagID(false)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Returns a list of detected AprilTag IDs.
     *
     * @return list of detected IDs (empty if none)
     */
    public List<Integer> getDetectedTagIDs() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            ids.add(detection.id);
        }
        return ids;
    }

    /**
     * Returns the ID of the first detected tag (if any).
     *
     * @return first tag ID, or -1 if none are detected
     */
    public int getFirstDetectedTagID() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }

    /**
     * Checks whether any AprilTags are currently detected.
     *
     * @return true if one or more tags detected
     */
    public boolean hasTags() {
        return !aprilTag.getDetections().isEmpty();
    }
}
