package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.LinearVelocityUnit;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;


public final class RobotConstants {
    // Change this when we want to do replay
    public static final Mode currentMode = Utils.isSimulation() ? Mode.SIM : Mode.REAL;

    public static enum Mode {
      REAL,
      SIM,
      REPLAY
    }

    public final class Logging {
        public static final boolean enabled = true;
        public static final boolean enableDSLogging = true;
    }
  
    public final class Driver {
        public static final double deadband = 0.1;
        public static final boolean useHeadingController = true;
    }

    public final class Drivetrain {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public final class HeadingController {
            public static final double kP = 10.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }

    public final class Vision {
        public static final boolean enabled = true;
        public static final String limelightName = "limelight";
        public static double maxRotationsPerSecond = 0.25;      // TODO: Tune this
    }

    public static AprilTagFieldLayout aprilTags;

    static {
        try {
            aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
