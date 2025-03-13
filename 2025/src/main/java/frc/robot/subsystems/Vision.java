package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.MathUtils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

@Logged
public class Vision extends SubsystemBase {
    @NotLogged private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    // Create instance variables for these so they get logged
    private long odometryUpdates = 0;
    private long odometryDiscards = 0;
    private Pose2d lastKnownPose = Pose2d.kZero;
    private LimelightHelpers.PoseEstimate lastPoseEstimate;
    private boolean rejectUpdate = false;
    private Rotation2d rotation = MathUtils.rotation.kZero;

    /** Creates a new VisionUpdate. */
    public Vision(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        // Tell Limelight what our current orientation is
        rotation = drivetrain.getPigeon2().getRotation2d();
        LimelightHelpers.SetRobotOrientation(limelightName, rotation.getDegrees(), 0, 0, 0, 0,
                0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 != null) {
            // Avoid logging errors by only updating when non-null
            lastPoseEstimate = mt2;
        }
        rejectUpdate = false;

        // If we don't see any tags, the pose can't be good
        if (mt2 == null || mt2.tagCount == 0) {
            rejectUpdate = true;
        }

        if (!rejectUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            lastKnownPose = mt2.pose;
            odometryUpdates++;
        } else {
            odometryDiscards++;
        }
    }
}