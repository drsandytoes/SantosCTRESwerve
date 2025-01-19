package frc.robot.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
    protected String limelightName;
    protected DoubleSupplier rotationsPerSecondSupplier;
    protected Supplier<Rotation2d> headingSupplier;

    public VisionIOLimelight(String limelightName, Supplier<Rotation2d> headingSupplier, DoubleSupplier rotationsPerSecondSupplier) {
        this.limelightName = limelightName;
        this.rotationsPerSecondSupplier = rotationsPerSecondSupplier;
        this.headingSupplier = headingSupplier;
    }   

    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.SetRobotOrientation(limelightName, headingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        boolean rejectPose = (megaTag2 == null ||
            Math.abs(rotationsPerSecondSupplier.getAsDouble()) > RobotConstants.Vision.maxRotationsPerSecond ||
            megaTag2.tagCount < 1);

        if (rejectPose) {
            inputs.poseEstimate = Optional.empty();
        } else {
            inputs.poseEstimate = Optional.of(megaTag2);
        }
    }

    
}
