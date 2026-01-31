package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;

public class VisionUpdate extends SubsystemBase {
    public enum VisionIMUMode {
        External(0),
        SeedInternal(1),
        InternalOnly(2),
        InternalPlusMT1(3),
        InternalPlusExternal(4),
        Unknown(5);

        private final int value;

        VisionIMUMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }
    }

    public enum VisionAlgorithm {
        MT1(1),
        MT2(2),
        Unknown(0);

        private final int value;

        VisionAlgorithm(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }
    }

    private String limelightName;
    private CommandSwerveDrivetrain drivetrain;
    private long acceptedUpdates = 0;
    private VisionAlgorithm algorithm = VisionAlgorithm.Unknown;
    private boolean isThrottled = false;
    private VisionIMUMode imuMode = VisionIMUMode.Unknown;

    public VisionUpdate(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
    }

    public void setIMUMode(VisionIMUMode mode) {
        imuMode = mode;
        LimelightHelpers.SetIMUMode(Constants.Vision.limelightName, mode.ordinal());
    }

    public void setVisionAlgorithm(VisionAlgorithm algorithm) {
        this.algorithm = algorithm;
    }

    public void setThrottled(boolean isThrottled) {
        this.isThrottled = isThrottled;
        if (isThrottled) {
            LimelightHelpers.SetThrottle(Constants.Vision.limelightName, 100);
        } else {
            LimelightHelpers.SetThrottle(Constants.Vision.limelightName, 0);
        }

    }

    @Override
    public void periodic() {
        super.periodic();

        var driveState = drivetrain.getState();
        Pose2d robotPose = driveState.Pose;
        Rotation2d heading = robotPose.getRotation();
        Pigeon2 pigeon = drivetrain.getPigeon2();
        Rotation2d pigeonYaw = pigeon.getRotation2d();
        Rotation2d rawHeading = driveState.RawHeading;

        LimelightHelpers.PoseEstimate poseEstimate;

        LimelightHelpers.SetRobotOrientation(limelightName, pigeonYaw.getDegrees(), 0, 0, 0, 0, 0);
        if (algorithm == VisionAlgorithm.MT2) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }
        boolean doRejectUpdate = false;

        DogLog.log("VisionUpdate/drivetrainHeading", heading);
        DogLog.log("VisionUpdate/pigeonYaw", pigeonYaw);
        DogLog.log("VisionUpdate/tagCount", poseEstimate != null ? poseEstimate.tagCount : 0);
        DogLog.log("VisionUpdate/drivetrainRawHeading", rawHeading);
        DogLog.log("VisionUpdate/drivetrainPose", robotPose);
        DogLog.log("VisionUpdate/algorithm", algorithm.getValue());
        DogLog.log("VisionUpdate/isThrottled", isThrottled);
        DogLog.log("VisionUpdate/IMUMode", imuMode.getValue());

        // If we don't see any tags, the pose can't be good
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);

            DogLog.log("VisionUpdate/pose", poseEstimate.pose);
            acceptedUpdates++;
        }

        DogLog.log("VisionUpdate/visionUpdates", acceptedUpdates);
        DogLog.log("VisionUpdate/acceptedUpdate", !doRejectUpdate);

    }
}
