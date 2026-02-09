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
    private Translation2d offset = Translation2d.kZero; // Relative to tag
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

    private Translation2d targetTranslation() {
        return targetPose.getTranslation();
    }

    public void setSimulatedRelativeLocation(Translation2d translationOffset) {
        offset = translationOffset;
    }

    public void setSimulatedRotation(Rotation2d rotation) {
        simulatedRotation = rotation;
    }

    public Pose2d simulatedPose() {
        Pose2d pose = new Pose2d(targetPose.getTranslation().plus(offset), simulatedRotation);
        DogLog.log("VisionSim/ActualPose", pose);

        return pose;
    }

    public PoseEstimate getMT1Estimate(Rotation2d gyroRotation) {
        // MT1 ignores the gyro, but has more error than MT2.
        Translation2d tagTranslation = targetTranslation();
        Translation2d robotPosition = tagTranslation.plus(offset);

        Pose2d pose = new Pose2d(robotPosition.getX(), robotPosition.getY(), simulatedRotation);

        // Apply some error
        pose = applyRandomError(pose, 0.25, 0.15);

        PoseEstimate estimate = new PoseEstimate();
        estimate.tagCount = 1;
        estimate.timestampSeconds = Timer.getFPGATimestamp();
        estimate.pose = pose;

        DogLog.log("VisionSim/MT1", estimate.pose);

        return estimate;
    }

    public PoseEstimate getMT2Estimate(Rotation2d gyroRotation) {
        // Compute a location relative to the tag that maintains the same distance, but obeys
        // the passed in gyroRotation.
        double distance = Math.hypot(offset.getX(), offset.getY());
        double angleRad = gyroRotation.getRadians();
        Translation2d tagTranslation = targetTranslation();
        Pose2d pose = new Pose2d(tagTranslation.getX() - distance * Math.cos(angleRad), 
            tagTranslation.getY() - distance * Math.sin(angleRad), 
            gyroRotation);

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
