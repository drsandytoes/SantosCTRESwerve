package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConstants;

public class LimelightIOReal implements LimelightIO {
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        var results = LimelightHelpers.getLatestResults(RobotConstants.Vision.limelightName);
        inputs.valid = results.valid;
        inputs.latency_capture = results.latency_capture;
        inputs.latency_pipeline = results.latency_pipeline;
        inputs.targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(RobotConstants.Vision.limelightName).getTranslation().getDistance(new Translation3d());
        if (inputs.valid) {
            inputs.botpose = LimelightHelpers.getBotPose2d_wpiBlue(RobotConstants.Vision.limelightName);
        }
        inputs.fiducialMarkerCount = results.targets_Fiducials.length;

        // We don't actually use these, but capture the IDs that were recognized
        inputs.fiducialMarkers = new double[inputs.fiducialMarkerCount];
        int idx = 0;
        for (var fiducial: results.targets_Fiducials) {
            inputs.fiducialMarkers[idx++] = fiducial.fiducialID;
        }
    }
}
