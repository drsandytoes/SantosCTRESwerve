package frc.robot.vision;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
    LimelightIO io;
    LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    CommandSwerveDrivetrain drivetrain;

    private Boolean enabled = false;
    private Boolean alwaysTrust = false;
    @AutoLogOutput
    private int fieldError = 0;
    @AutoLogOutput
    private int distanceError = 0;

    // Create a bounding box for the field so that we can reject obvious wrong
    // locations. We can assume that the
    // robot didn't break through the field barrier, and even if it did, we'd be
    // quickly disabled by FMS...
    private static final RectanglePoseArea field = new RectanglePoseArea(new Translation2d(0.0, 0.0),
            new Translation2d(16.54, 8.02));

    /** Creates a new Limelight. */
    public Limelight(LimelightIO io, CommandSwerveDrivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);

        // We purposefully don't add a dependency on the drivetrain because we don't
        // need exclusive access to it
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (enabled) {
            double confidence = 1 - ((inputs.targetDistance - 1) / 6);
            if (inputs.valid) {
                // Reject position updates that put us outside of the field
                if (field.isPoseWithinArea(inputs.botpose)) {
                    // Reject updates that are more than 0.5m unless we're in always-trust mode, or
                    // we can see more than
                    // one April tag.
                    if (drivetrain.getState().Pose.getTranslation().getDistance(inputs.botpose.getTranslation()) < 0.5
                            || alwaysTrust
                            || inputs.fiducialMarkerCount > 1) {
                        drivetrain.addVisionMeasurement(
                                inputs.botpose,
                                Timer.getFPGATimestamp()
                                        - (inputs.latency_capture / 1000.0)
                                        - (inputs.latency_pipeline / 1000.0),
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
