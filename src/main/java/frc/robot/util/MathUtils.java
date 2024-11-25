package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class MathUtils {
    public final class rotation {
        static public final Rotation2d zero = Rotation2d.fromDegrees(0);
        static public final Rotation2d quarter = Rotation2d.fromDegrees(90);
        static public final Rotation2d half = Rotation2d.fromDegrees(180);
        static public final Rotation2d threeQuarters = Rotation2d.fromDegrees(270);
    }
    public final class translation {
        static public final Translation2d zero = new Translation2d();
    }
    public final class pose {
        static public final Pose2d zero = new Pose2d();
    }
}
