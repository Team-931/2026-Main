package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGeneratorHack;

/** A class to encapsulate the pre-generated drive trajectories and any landmarks 
 * within them.
 * <p>Note: the {@link Pose2d} objectss in here have {@link Rotation2d} components
 * that refer to ** direction to move **, not direction to face. Direction to face
 * will be controlled separately.
 * In the {@link Translation2d} objects:
 *  positive x is forward
 *  positive y is left
 */

final class OurTrajectories {
    static TrajectoryConfig config = new TrajectoryConfig(1, 2);
    static List<TrajectoryGeneratorHack.Landmark> landmarks = List.of(new TrajectoryGeneratorHack.Landmark(3.3), new TrajectoryGeneratorHack.Landmark(1.2));
    static Pose2d startPt = new Pose2d(1, 3, Rotation2d.kZero);
    static Trajectory circleTrajectory = 
        TrajectoryGeneratorHack.generateTrajectory(
            startPt, // start position and heading
            List.of(
                startPt.getTranslation().plus(new Translation2d(1, 1)), // first intermediate way point ...
                startPt.getTranslation().plus(new Translation2d(0, 2)),
                startPt.getTranslation().plus(new Translation2d(-1, 1))), 
            startPt, // end position and heading
            config, landmarks);
}
