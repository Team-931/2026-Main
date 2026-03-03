package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Note: the {@link Pose2d} objectss in here have {@link Rotation2d} components
 * that refer to ** direction to move **, not direction to face. Direction to face
 * will be controlled separately.
 * In the {@link Translation2d} objects:
 *  positive x is forward
 *  positive y is left
 */

final class OurTrajectories {
    static TrajectoryConfig config = new TrajectoryConfig(1, 2);
    static List<TrajectoryGenerator.Landmark> landmarks = List.of(new TrajectoryGenerator.Landmark(3.9), new TrajectoryGenerator.Landmark(1.2));
    static Trajectory circleTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            Pose2d.kZero, // start position and heading
            List.of(
                new Translation2d(1, 1), // first intermediate way point ...
                new Translation2d(0, 2),
                new Translation2d(-1, 1)), 
            Pose2d.kZero, // end position and heading
            config, landmarks);
}
