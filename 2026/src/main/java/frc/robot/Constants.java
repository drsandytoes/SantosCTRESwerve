package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
    public static final class Vision {
        public static final String limelightName = "limelight";

        // If true, Vision will set the pose of the robot when disabled 
        // instead of applying vision updates. This will allow Vision to be
        // used to detect the placement of the robot when disabled.
        public static final boolean trustVisionPoseWhileDisabled = true;
    }

    public static final class VisionStateMachine {
        public static final double detectDelay = 4;
        public static final double postCommitDelay = 0.5;
    }

    public static final class LED {
        public static final int port = 0;
        public static final int stringLength = 16 * 16 * 3;
    }
    
}
