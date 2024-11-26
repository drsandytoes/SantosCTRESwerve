package frc.robot.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

public interface VisionIO {
    public static class VisionIOInputs implements LoggableInputs {
        Optional<PoseEstimate> poseEstimate;

        @Override
        public void toLog(LogTable table) {
            if (!poseEstimate.isEmpty()) {
                var estimate = poseEstimate.get();
                table.put("pose", LimelightHelpers.pose2dToArray(estimate.pose));
                table.put("timestampSeconds", estimate.timestampSeconds);
                table.put("tagCount", estimate.tagCount);
                table.put("avgTagDist", estimate.avgTagDist);

                // Should be the same as tagCount, but protect ourselves against bad indexing
                var fiducialCount = estimate.rawFiducials.length; 
                for (var i = 0; i < fiducialCount; i++) {
                    RawFiducial fiducial = estimate.rawFiducials[i];
                    table.put("fiducial/" + i + "/id", fiducial.id);
                    table.put("fiducial/" + i + "/txnc", fiducial.txnc);
                    table.put("fiducial/" + i + "/tync", fiducial.tync);
                    table.put("fiducial/" + i + "/ta", fiducial.ta);
                    table.put("fiducial/" + i + "/distToCamera", fiducial.distToCamera);
                    table.put("fiducial/" + i + "/distToRobot", fiducial.distToRobot);
                    table.put("fiducial/" + i + "/ambiguity", fiducial.ambiguity);
                }
            }
            table.put("valid", !poseEstimate.isEmpty());
        }
    
        @Override
        public void fromLog(LogTable table) {
            Pose2d pose = LimelightHelpers.toPose2D(table.get("pose", new double[] {}));
            double timestampSeconds = table.get("timestampSeconds", 0.0);
            int tagCount = table.get("tagCount", 0);
            double avgTagDist = table.get("avgTagDist", 0.0);

            // Read in the fiducials
            var rawFiducials = new RawFiducial[tagCount];
            for (var i = 0; i < tagCount; i++) {
                int id = table.get("fiducial/" + i + "/id", 0);
                double txnc = table.get("fiducial/" + i + "/txnc", 0.0);
                double tync = table.get("fiducial/" + i + "/tync", 0.0);
                double ta = table.get("fiducial/" + i + "/ta", 0.0);
                double distToCamera = table.get("fiducial/" + i + "/distToCamera", 0.0);
                double distToRobot = table.get("fiducial/" + i + "/distToRobot", 0.0);
                double ambiguity = table.get("fiducial/" + i + "/ambiguity", 0.0);
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }

            PoseEstimate poseEstimate =
                new PoseEstimate(
                    pose,
                    timestampSeconds,
                    0,
                    tagCount,
                    0,
                    avgTagDist,
                    0,
                    rawFiducials);

            if (table.get("valid", false)) {
                this.poseEstimate = Optional.of(poseEstimate);
            } else {
                this.poseEstimate = Optional.empty();
            }
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}
}
