package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.MathUtils;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    /** Creates a new VisionUpdate. */
    public Vision(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        // CTRE doesn't actually reset the gyro when you reset the pose, so we need to get the heading out of the pose, and not the 
        // Pigeon directly.
        var drivetrainState = drivetrain.getState();
        Rotation2d pigeonYaw = drivetrain.getPigeon2().getRotation2d();
        Rotation2d rawHeading = drivetrainState.RawHeading;
        Pose2d robotPose = drivetrainState.Pose;
        Rotation2d fieldHeading = robotPose.getRotation();
        double rotationsPerSecond = Units.radiansPerSecondToRotationsPerMinute(drivetrainState.Speeds.omegaRadiansPerSecond);

        // Tell Limelight what our current orientation is
        LimelightHelpers.SetRobotOrientation(limelightName, fieldHeading.getDegrees(), 0, 0, 0, 0,
                0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        boolean rejectUpdate = false;

        DogLog.log("Vision/hasMT2", mt2 != null);
        DogLog.log("Vision/pigeonYaw", pigeonYaw);
        DogLog.log("Vision/tagCount", mt2 != null ? mt2.tagCount : 0);
        DogLog.log("Vision/drivetrainRawHeading", rawHeading);
        DogLog.log("Vision/drivetrainPose", robotPose);
        DogLog.log("Vision/fieldHeading", fieldHeading);
        DogLog.log("Vision/rotationsPerSecond", rotationsPerSecond);

        // If we don't see any tags, the pose can't be good
        if (mt2 == null || mt2.tagCount == 0) {
            rejectUpdate = true;
        }
        if (!rejectUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            DogLog.log("Vision/mt2Pose", mt2.pose);
        }

        DogLog.log("Vision/acceptedUpdate", !rejectUpdate);

    }
}