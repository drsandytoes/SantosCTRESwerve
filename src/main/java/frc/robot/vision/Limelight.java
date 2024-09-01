package frc.robot.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  private String limelightName = "limelight";
  private Boolean enabled = false;
  private Boolean alwaysTrust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;

  // Create a bounding box for the field so that we can reject obvious wrong locations. We can assume that the
  // robot didn't break through the field barrier, and even if it did, we'd be quickly disabled by FMS...
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);

    // We purposefully don't add a dependency on the drivetrain because we don't need exclusive access to it
  }

  @Override
  public void periodic() {
    if (enabled) {
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getTranslation().getDistance(new Translation3d());
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.LimelightResults result =
          LimelightHelpers.getLatestResults(limelightName);
      if (result.valid) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);

        // Reject position updates that put us outside of the field
        if (field.isPoseWithinArea(botpose)) {
          // Reject updates that are more than 0.5m unless we're in always-trust mode, or we can see more than 
          // one April tag.
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || alwaysTrust
              || result.targets_Fiducials.length > 1) {
            drivetrain.addVisionMeasurement(
                botpose,
                Logger.getRealTimestamp() // MDS TODO: Move this to a capture input stage
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(confidence, confidence, .01));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }
    }
  }

  public void useLimelight(boolean enable) {
    this.enabled = enable;
  }

  public void trustLimelight(boolean trust) {
    this.alwaysTrust = trust;
  }
}
