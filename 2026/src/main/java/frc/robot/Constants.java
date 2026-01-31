package frc.robot;

public final class Constants {
    public static final class Vision {
        public static final String limelightName = "limelight";

        // If true, Vision will set the pose of the robot when disabled 
        // instead of applying vision updates. This will allow Vision to be
        // used to detect the placement of the robot when disabled.
        public static final boolean trustVisionPoseWhileDisabled = true;
    }

    public static final class VisionStateMachine {
        public static final int detectDelay = 5;
    }
    
}
