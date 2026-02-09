package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LED.SimpleLEDPatternApplier;
import frc.robot.subsystems.LED.SimpleLEDSubsystem;
import frc.robot.subsystems.VisionUpdate.VisionAlgorithm;
import frc.robot.subsystems.VisionUpdate.VisionIMUMode;
import frc.robot.subsystems.VisionUpdate.VisionTrustLevel;

/**
 * This class manages the configuration of the VisionUpdate class as the system
 * progresses through different states. The basic idea is as follows.
 * 
 * When the robot enters disabled mode, we should: [BOOT state]
 * 1. Set the Limelight to throttled mode to avoid overheating
 * 2. Set the Limelight to syncing mode to that it's internal IMU matches the
 * Pigeon
 * 
 * In addition, once at startup, and whenever the chosen auto path changes, we
 * should: [DETECT state]
 * 1. Read the currently selected auto, extracting the starting pose
 * 2. Set the starting pose (translation + rotation) in the drivetrain's pose
 * estimator
 * 3. Start a timer for some predetermined amount of time
 * 4. Peform vision updates using MegaTag 1
 * 
 * When the timer expires, or perhaps when "convergence is achieved": [COMMIT
 * state]
 * 1. Capture the current pose from the pose estimator
 * 2. Reset the Pigeon's yaw to match that pose
 * 3. Reset the pose estimator's pose to the captured pose (internally resetting
 * any yaw-pose offset CTRE maintains)
 * 4. Switch to using MegaTag 2 for pose updates
 * 
 * When the robot becomes enabled (whether auto or teleop): [ENABLED state]
 * 1. Set the Limelight to using fused IMU mode (both internal and external IMU)
 * 2. Set the Limelight to full speed capture (not throttled)
 * 3. If we hadn't reached the FINE TUNE state, allow Choreo to set the robot
 * pose when auto starts; otherwise ignore it
 * 
 * Essentially the idea is to use the initial pose from auto (or some test
 * dashboard when testing) to set the initial pose
 * of the robot. But then to immediately start using MT1 updates (which ignore
 * the gyro) to attempt to refine that position
 * to adjust for errors in initial translation AND rotation. After some period
 * of time (or possibly after seeing MT1 updates
 * that are "close enough" to the current position estimate), stop performing
 * MT1 updates, and capture the current pose.
 * Update the Pigeon yaw (which will sync to the LL IMU) to match our refined
 * pose, and then reset the pose to that refined
 * pose. The latter is needed because CTRE maintains an offset between the
 * Pigeon yaw and the rotation of the robot, and
 * that needs to be reset after forcing the Pigeon's yaw. Then, until the robot
 * is enabled, use MT2 pose updates to further
 * refine position, but at that point, we won't be able to correct for rotation
 * error.
 * 
 * By time the robot is enabled, the Pigeon and LL IMU should be as close as
 * possible to our correct rotation, and the LL
 * should be performing MT2 updates. Once enabled, we need to switch the LL into
 * fused IMU mode and unthrottle it. We want to
 * prevent Choreo from resetting the pose when auto starts because we've already
 * adjusted it.
 * 
 * If the robot is enabled before we got to the COMMIT state, our process didn't
 * complete. At that point, we should assume
 * that all failed. I think we'll want to allow Choreo to reset the robot pose
 * when auto starts because we won't trust the
 * current pose estimator. We also need to make sure to reset the Pigeon to
 * match. Because the LL IMU won't have been synced
 * at that point, we need to leave the LL configured for external-only mode.
 * 
 */

public class VisionConfigurationControl extends SubsystemBase {
    enum ConfigurationState {
        BOOT,
        DETECT,
        COMMIT,
        DONE,
        ENABLED,
    }

    private ConfigurationState currentState = ConfigurationState.BOOT;
    
    private VisionUpdate visionUpdateSubsystem;
    private CommandSwerveDrivetrain drivetrain;
    private SimpleLEDPatternApplier ledSubsystem;

    private Pose2d startingPose = Pose2d.kZero;
    private Pose2d detectedPose = Pose2d.kZero;
    private Timer delayTimer = new Timer();
    private boolean localizationCompleted = false;

    // Visualization patterns
    private final LEDPattern offPattern = LEDPattern.solid(Color.kBlack);
    private final LEDPattern detectPattern = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.25));
    private final LEDPattern commitPattern = LEDPattern.solid(Color.kBlue);
    private final LEDPattern donePattern = LEDPattern.solid(Color.kGreen);
    private final LEDPattern enabledPattern = LEDPattern.solid(Color.kWhite);

    public VisionConfigurationControl(VisionUpdate visionUpdateSubsystem, CommandSwerveDrivetrain drivetrain, SimpleLEDPatternApplier ledSubsystem) {
        this.visionUpdateSubsystem = visionUpdateSubsystem;
        this.drivetrain = drivetrain;
        this.ledSubsystem = ledSubsystem;

        moveToState(ConfigurationState.BOOT);
    }

    @Override
    public void periodic() {
        super.periodic();

        DogLog.log("VisionControl/startingPose", startingPose);
        DogLog.log("VisionControl/state", currentState.ordinal());
        DogLog.log("VisionControl/detectedPose", detectedPose);

        // Handles things we do every time we're in this state
        switch (currentState) {
            case BOOT:
                moveToState(ConfigurationState.DETECT);
                break;

            case DETECT:
                // Move on to COMMIT state if enough time has passed. Might want to add a
                // convergence criteria as well.
                if (delayTimer.hasElapsed(Constants.VisionStateMachine.detectDelay)) {
                    delayTimer.stop();
                    moveToState(ConfigurationState.COMMIT);
                }
                break;

            case COMMIT:
                // Move on to DONE state after a short delay to prevent using the Pigeon before
                // the yaw has truly reset.
                if (delayTimer.hasElapsed(Constants.VisionStateMachine.postCommitDelay)) {
                    delayTimer.stop();
                    moveToState(ConfigurationState.DONE);
                }
                break;

            case DONE:
            case ENABLED:
                break;
        }

    }

    /**
     * Should be called when the robot becomes enabled or becomes disabled.
     * @param isEnabled whether the robot is enabled
     */
    public void robotEnabled(boolean isEnabled) {
        if (isEnabled) {
            moveToState(ConfigurationState.ENABLED);
        }

        // Should we go back to decting if we become disabled? Probably useful for testing, but what if it
        // happens during a match?
    }

    /**
     * Helper method. Can be used to set the starting pose when a Choreo auto starts. This method will 
     * ignore the reset if the localization completed, and will take care of resetting the gyro and pose
     * to the starting pose if the process did not complete.
     * @param pose starting pose from the auto
     */
    public void resetPoseForChoreoStart(Pose2d pose) {
        if (!isFinished()) {
            resetPigeonAndPose(pose);
        }
    }

    /**
     * Should be called when a new starting pose has been selected (while disabled), generally as a result of
     * choosing a new auto from the auto chooser.
     * @param pose the starting pose of the robot
     */
    public void setNewStartingPose(Pose2d pose) {
        startingPose = pose;

        reset(false);
    }

    /**
     * Can be called to reset the state machine. If the robot has already been enabled, this will be ignored
     * unless the force flag is set.
     * @param force Force the state machine to reset even if the robot has already been enabled
     */
    public void reset(boolean force) {
        if (currentState != ConfigurationState.ENABLED && !force) {
            moveToState(ConfigurationState.BOOT);
        }
    }

    /**
     * Indicates whether the vision localization process has completed.
     * @return whether vision localization process completed
     */
    public boolean isFinished() {
        return localizationCompleted;
    }

    private void moveToState(ConfigurationState newState) {
        // Handle entry into the new state
        switch (newState) {
            case BOOT:
                break;

            case DETECT:
                localizationCompleted = false;
                visionUpdateSubsystem.setThrottled(true);
                visionUpdateSubsystem.setVisionAlgorithm(VisionAlgorithm.MT1);
                visionUpdateSubsystem.setIMUMode(VisionIMUMode.SeedInternal);
                visionUpdateSubsystem.setTrustLevel(VisionTrustLevel.MT1_PRE_MATCH);
                detectedPose = Pose2d.kZero;
                drivetrain.resetPose(startingPose);
                delayTimer.reset();
                delayTimer.start();
                break;

            case COMMIT:
                // Reset Pigeon and pose estimate to current estimate
                detectedPose = drivetrain.getState().Pose;
                resetPigeonAndPose(detectedPose);

                // Disable vision updates for a brief period
                visionUpdateSubsystem.setVisionAlgorithm(VisionAlgorithm.DISABLED);
                delayTimer.reset();
                delayTimer.start();

                localizationCompleted = true;
                break;

            case DONE:
                // Resume vision updates with MT2 after a brief settling period
                visionUpdateSubsystem.setVisionAlgorithm(VisionAlgorithm.MT2);
                visionUpdateSubsystem.setTrustLevel(VisionTrustLevel.MT2_PRE_MATCH);
                break;

            case ENABLED:
                // Transition to enabled
                visionUpdateSubsystem.setThrottled(false);
                visionUpdateSubsystem.setTrustLevel(VisionTrustLevel.MT2_MATCH);

                if (isFinished()) {
                    // We finished, so we can safely use the fused IMU mode
                    visionUpdateSubsystem.setIMUMode(VisionIMUMode.InternalPlusExternal);
                } else {
                    // We can move to MT2, but we need to rely on the Pigeon, as the LL IMU didn't get a chance to
                    // sync.
                    visionUpdateSubsystem.setIMUMode(VisionIMUMode.External);
                }
                break;

        }

        currentState = newState;
        updateVisualization();
    }

    private void updateVisualization() {
        LEDPattern pattern = null;
        switch (currentState) {
            case BOOT:
                pattern = offPattern;
                break;
            case DETECT:
                pattern = detectPattern;
                break;
            case COMMIT:
                pattern = commitPattern;
                break;
            case DONE:
                pattern = donePattern;
                break;
            case ENABLED:
                pattern = enabledPattern;
                break;
        }

        if (pattern != null) {
            ledSubsystem.applyPattern(pattern);
        } else {
            System.out.println("NULL pattern to apply!!");
        }
    }

    private void resetPigeonAndPose(Pose2d pose) {
        Pigeon2 pigeon = drivetrain.getPigeon2();
        pigeon.setYaw(pose.getRotation().getDegrees());
        pigeon.getYaw().refresh();

        double degrees = pigeon.getYaw().getValueAsDouble();
        Rotation2d rotation = pigeon.getRotation2d();
        DogLog.log("VisionControl/PigeonResetYaw", degrees);
        DogLog.log("VisionControl/PigeonResetRotation", rotation.getDegrees());
        DogLog.log("VisionControl/PigeonResetTarget", pose.getRotation().getDegrees());


        drivetrain.resetPose(pose);

        DogLog.log("VisionControl/ResetPose", pose);
    }

}
