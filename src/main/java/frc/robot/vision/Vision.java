package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.VisionUtils.TimestampedVisionUpdate;
import frc.robot.util.RectanglePoseArea;
import frc.robot.vision.VisionIO.VisionIOInputs;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO ios[];
    private VisionIOInputs inputs[];
    private Consumer<TimestampedVisionUpdate> visionUpdateConsumer;

    private Boolean enabled = false;
    private Boolean alwaysTrust = false;

    // Keep track of last time we got valid input from a given camera, and the last
    // time
    // we saw a given tag.
    private Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    // Create a bounding box for the field so that we can reject obvious wrong
    // locations. We can assume that the robot didn't break through the field
    // barrier, and even if it did, we'd be quickly disabled by FMS so it doesn't
    // matter anymore.
    private static final RectanglePoseArea field = new RectanglePoseArea(new Translation2d(0.0, 0.0),
            new Translation2d(16.54, 8.02));

    /**
     * Creates a new Vision processing engine. Supports multiple camera inputs
     * 
     * @param drivetrain The CommandSwerveDrivetrain drivetrain
     * @param io...      Any number of vision inputs
     */
    public Vision(Consumer<TimestampedVisionUpdate> visionUpdateConsumer, VisionIO... io) {
        inputs = new VisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputs();
        }

        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }

        // Create map of last detection times for tags
        RobotConstants.aprilTags.getTags().forEach(tag -> lastTagDetectionTimes.put(tag.ID, 0.0));

        this.visionUpdateConsumer = visionUpdateConsumer;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + Integer.toString(i), inputs[i]);
        }

        if (enabled) {
            List<TimestampedVisionUpdate> visionUpdates = processPoseEstimates();
            sendResultsToPoseEstimator(visionUpdates);
        }
    }

    private List<TimestampedVisionUpdate> processPoseEstimates() {
        List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
        for (int ioIndex = 0; ioIndex < ios.length; ioIndex++) {
            if (!inputs[ioIndex].poseEstimate.isEmpty()) {
                var estimate = inputs[ioIndex].poseEstimate.get();
                if (shouldSkipPoseEstimate(estimate)) {
                    continue;
                }

                double timestamp = estimate.timestampSeconds;
                Pose2d robotPose = estimate.pose;
                double xyStdDev = calculateXYStdDev(estimate, estimate.tagCount);
                double thetaStdDev = 9999999; // Limelight sample code value

                visionUpdates.add(new TimestampedVisionUpdate(timestamp, robotPose,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
            }
        }

        return visionUpdates;
    }

    private void sendResultsToPoseEstimator(List<TimestampedVisionUpdate> updates) {
        for (var update : updates) {
            visionUpdateConsumer.accept(update);
        }
    }

    /**
     * Controls whether this subsystem processes any data. Vision measurements will still
     * be logged regardless.
     * 
     * @param enable Enables vision processing if true.
     */
    public void useVision(boolean enable) {
        this.enabled = enable;
    }

    /**
     * Determines whether vision updates should always be trusted, or whether certain sanity checks
     * should be in place.
     * 
     * @param trust Always trusts vision measurements when true.
     */
    public void trustVision(boolean trust) {
        this.alwaysTrust = trust;
    }

    /**
     * Check if the pose estimate should be skipped.
     *
     * @param poseEstimate The pose estimate
     * @return True if the pose estimate should be skipped, false otherwise
     */
    private boolean shouldSkipPoseEstimate(PoseEstimate poseEstimate) {
        return poseEstimate.tagCount < 1
                || poseEstimate.pose == null
                || alwaysTrust // If true, remaining checks ignored
                || !field.isPoseWithinArea(poseEstimate.pose);
    }

    /**
     * Calculate the standard deviation of the x and y coordinates. 
     * Not sure how much this matters, other than we want the stddev to increase
     * exponentially as the avg distance increases. Limelight stample code just
     * uses a hard-coded value of 0.7
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize  The number of detected tag poses
     * @return The standard deviation of the x and y coordinates
     */
    private double calculateXYStdDev(PoseEstimate poseEstimates, int tagPosesSize) {
        return 0.1 * Math.pow(poseEstimates.avgTagDist, 2.0) / tagPosesSize;
    }

}
