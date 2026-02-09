package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;

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
        DISABLED(3),
        Unknown(0);

        private final int value;

        VisionAlgorithm(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }
    }

    public enum VisionTrustLevel {
        // These values trust Vision significantly because they'll be used when the Robot is 
        // running into things. For MT2, there's no point in trusting the angle because it's 
        // always angle-locked.
        MT1_PRE_MATCH(VecBuilder.fill(0.15, 0.15, 0.08)),
        MT2_PRE_MATCH(VecBuilder.fill(0.10, 0.10, 999999)),

        // Trust vision less during the match. It might make sense to have different values
        // here low, medium, and high velocity. Also might want to scale by tag distance. Maybe
        // something like: 0.25 + 0.15 * <distance>.
        // May also want to consider reported tag ambiguity.
        MT2_MATCH(VecBuilder.fill(0.40, 0.40, 999999)),

        // This is the default value suggested by Limelight.
        LL_DEFAULT(VecBuilder.fill(.7, .7, 9999999));

        private final Vector<N3> value;

        VisionTrustLevel(Vector<N3> value) {
            this.value = value;
        }

        public Vector<N3> getValue() {
            return value;
        }
    }


    private String limelightName;
    private CommandSwerveDrivetrain drivetrain;
    private long acceptedUpdates = 0;
    private VisionAlgorithm algorithm = VisionAlgorithm.Unknown;
    private boolean isThrottled = false;
    private VisionIMUMode imuMode = VisionIMUMode.Unknown;
    private VisionTrustLevel trustLevel = VisionTrustLevel.LL_DEFAULT;

    private VisionSimulation sim;

    public VisionUpdate(CommandSwerveDrivetrain drivetrain, String limelightName, VisionSimulation visionSim) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        sim = visionSim;
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

    public void setTrustLevel(VisionTrustLevel trustLevel) {
        this.trustLevel = trustLevel;
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

        // Do this regardless; even when using MT1, we might be syncing the LL IMU to the Pigeon
        LimelightHelpers.SetRobotOrientation(limelightName, pigeonYaw.getDegrees(), 0, 0, 0, 0, 0);

        // Fetch both poses so we can put them on the dashboard.
        PoseEstimate mt1Estimate = getMT1Estimate(pigeonYaw);
        PoseEstimate mt2Estimate = getMT2Estimate(pigeonYaw);
        PoseEstimate poseEstimate = (algorithm == VisionAlgorithm.MT2) ? mt2Estimate : mt1Estimate;

        boolean doRejectUpdate = (algorithm == VisionAlgorithm.DISABLED);

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
            drivetrain.setVisionMeasurementStdDevs(trustLevel.getValue());
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);

            DogLog.log("VisionUpdate/pose", poseEstimate.pose);
            acceptedUpdates++;
        }

        DogLog.log("VisionUpdate/visionUpdates", acceptedUpdates);
        DogLog.log("VisionUpdate/acceptedUpdate", !doRejectUpdate);

    }

    PoseEstimate getMT1Estimate(Rotation2d rotation) {
        if (Robot.isSimulation()) {
            return sim.getMT1Estimate(rotation);
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }
    }

    PoseEstimate getMT2Estimate(Rotation2d rotation) {
        if (Robot.isSimulation()) {
            return sim.getMT2Estimate(rotation);
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }
    }

}
