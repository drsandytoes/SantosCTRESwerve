package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class MathUtils {
    public final class rotation {
        static public final Rotation2d kZero = Rotation2d.kZero;
        static public final Rotation2d kQuarter = Rotation2d.kCCW_Pi_2;
        static public final Rotation2d kHalf = Rotation2d.kPi;
        static public final Rotation2d kThreeQuarters = Rotation2d.kCW_Pi_2;
    }
    public final class translation {
        static public final Translation2d kZero = new Translation2d();
    }
    public final class pose {
        static public final Pose2d kZero = new Pose2d();
    }
}
