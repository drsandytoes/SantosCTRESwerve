package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionUtils {
    /**
     * Represents a timestamped vision update with pose and standard deviations. We use this 
     * because this is what the odometry object wants to take as input.
     *
     * @param timestamp The timestamp of the vision update.
     * @param pose      The pose estimate.
     * @param stdDevs   The standard deviations matrix.
     */
    public record TimestampedVisionUpdate(
            /** The timestamp of the vision update. */
            double timestamp,
            /** The pose estimate. */
            Pose2d pose,
            /** The standard deviations matrix. */
            Matrix<N3, N1> stdDevs) {

        /**
         * Returns a string representation of this vision update.
         *
         * @return The string representation.
         */
        @Override
        public String toString() {
            return "VisionUpdate{"
                    + "timestamp="
                    + Double.toString(timestamp)
                    + ", pose="
                    + pose.toString()
                    + ", stdDevs="
                    + stdDevs.toString()
                    + '}';
        }
    }
}
