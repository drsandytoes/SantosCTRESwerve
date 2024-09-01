package frc.robot;

import frc.robot.generated.TunerConstants;

public final class RobotConstants {
    public final class Driver {
        public static final double deadband = 0.1;
    }

    public final class Drivetrain {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public final class HeadingController {
            public static final double kP = 10.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }
}
