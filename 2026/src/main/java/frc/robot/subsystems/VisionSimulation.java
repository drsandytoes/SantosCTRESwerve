package frc.robot.subsystems;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSimulation {
    private AprilTagFieldLayout fieldLayout;

    // Info about the tag we see on the field
    private int targetTag = 1;
    private Pose2d targetPose = Pose2d.kZero;

    // Info about where we *really* are
    private double simulatedDistance = 0.0;
    private Rotation2d simulatedRotation = Rotation2d.kZero; // Relative to field

    public VisionSimulation() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // WPILib provides the file automatically
        } catch (Exception e) {
        }
    }

    public void setTargetTag(int tag) {
        targetTag = tag;
        targetPose = targetTagPose();

        DogLog.log("VisionSim/TargetTag", targetPose);
        logSimulatedPose();
    }

    private Pose2d targetTagPose() {
        Pose2d pose = Pose2d.kZero;
        Optional<Pose3d> tagInfo = fieldLayout.getTagPose(targetTag);
        if (tagInfo.isPresent()) {
            Pose3d tagPose3 = tagInfo.get();
            pose = new Pose2d(tagPose3.getMeasureX(), tagPose3.getMeasureY(), 
                tagPose3.getRotation().toRotation2d());
        }

        return pose;
    }

    public void setSimulatedDistance(double distance) {
        simulatedDistance = distance;
        logSimulatedPose();
    }

    public void setSimulatedRotation(Rotation2d rotation) {
        simulatedRotation = rotation;
        logSimulatedPose();
    }

    public void logSimulatedPose() {
        Pose2d pose = robotPoseForTag(targetPose, simulatedRotation, simulatedDistance);
        DogLog.log("VisionSim/ActualPose", pose);
    }

    public PoseEstimate getMT1Estimate(Rotation2d gyroRotation) {
        // MT1 ignores the gyro, but has more error than MT2.
        Pose2d pose = robotPoseForTag(targetPose, simulatedRotation, simulatedDistance);

        // Apply some error
        pose = applyRandomError(pose, 0.25, 0.15);

        PoseEstimate estimate = new PoseEstimate();
        estimate.tagCount = 1;
        estimate.timestampSeconds = Timer.getFPGATimestamp();
        estimate.pose = pose;

        DogLog.log("VisionSim/MT1", estimate.pose);

        return estimate;
    }

    /**
     * Given a tag pose, a distance from that tag, and a heading of a robot, determine the pose
     * of the robot. Like MegaTag2, this doesn't consider whether it would be possible to see
     * the tag from the returned position.
     * @param tagPose Pose of the tag being seen. Only the translation component is considered.
     * @param robotRotation The robot's heading in field coordinates
     * @param distanceFromTag The robot's distance from the tag
     * @return pose of the robot meeting those conditions
     */
    private Pose2d robotPoseForTag(Pose2d tagPose, Rotation2d robotRotation, double distanceFromTag) {
        double angleRad = robotRotation.getRadians();
        Translation2d tagTranslation = tagPose.getTranslation();
        Pose2d pose = new Pose2d(tagTranslation.getX() - distanceFromTag * Math.cos(angleRad), 
            tagTranslation.getY() - distanceFromTag * Math.sin(angleRad), 
            robotRotation);

        return pose;
    }

    public PoseEstimate getMT2Estimate(Rotation2d gyroRotation) {
        // Compute a location relative to the tag that maintains the same distance, but obeys
        // the passed in gyroRotation.
        Pose2d pose = robotPoseForTag(targetPose, gyroRotation, simulatedDistance);

        // Apply some error
        pose = applyRandomError(pose, 0.15, 0.0);

        PoseEstimate estimate = new PoseEstimate();
        estimate.pose = pose;
        estimate.tagCount = 1;
        estimate.timestampSeconds = Timer.getFPGATimestamp();
        estimate.isMegaTag2 = true;

        DogLog.log("VisionSim/MT2", pose);

        return estimate;
    }

    private double random(double magnitude) {
        double r = (Math.random() - 0.5) * 2.0; // Convert [0, 1.0] => [-1.0, 1.0]
        return r * magnitude;
    }

    private Pose2d applyRandomError(Pose2d value, double maxTranslationError, double maxAngularRadError) {
        return new Pose2d(value.getX() + random(maxTranslationError), value.getY() + random(maxTranslationError),
            Rotation2d.fromRadians(value.getRotation().getRadians() + random(maxAngularRadError)));
    }

}
