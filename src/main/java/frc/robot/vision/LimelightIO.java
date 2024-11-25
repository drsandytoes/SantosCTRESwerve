package frc.robot.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.MathUtils;

public interface LimelightIO {
    @AutoLog
    public static class LimelightIOInputs {
        double targetDistance = 0.0;
        Pose2d botpose = MathUtils.pose.zero;

        // Things we use out of the results object
        boolean valid = false;
        double latency_capture = 0.0;
        double latency_pipeline = 0.0;
        int fiducialMarkerCount = 0;
        double fiducialMarkers[] = new double[0]; // ArrayList would be nice here, but I'm not sure AutoLog can do those
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LimelightIOInputs inputs) {}
}
