// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Hacked by Philp Harrington

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.spline.SplineParameterizer.LandmarkInfo;
import edu.wpi.first.math.spline.SplineParameterizer.MalformedSplineException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;

/** Helper class used to generate trajectories with various constraints.
 * <p> Harrington's added members are:
 * <p> {@link Landmark},
 * <p> {@link TrajectoryGenerator#generateTrajectory(Pose2d, List, Pose2d, TrajectoryConfig, Collection)},
 * <p> {@link TrajectoryGenerator#generateTrajectory(edu.wpi.first.math.spline.Spline.ControlVector, List,
 *  edu.wpi.first.math.spline.Spline.ControlVector, TrajectoryConfig, Collection)},
 * <p> and {@link TrajectoryGenerator#splinePointsFromSplines(Spline[], LandmarkInfo[])}.
 */
public final class TrajectoryGenerator {
  private static final Transform2d kFlip = new Transform2d(Translation2d.kZero, Rotation2d.kPi);

  private static final Trajectory kDoNothingTrajectory =
      new Trajectory(List.of(new Trajectory.State()));
  private static BiConsumer<String, StackTraceElement[]> errorFunc;

  /** Private constructor because this is a utility class. */
  private TrajectoryGenerator() {}

  private static void reportError(String error, StackTraceElement[] stackTrace) {
    if (errorFunc != null) {
      errorFunc.accept(error, stackTrace);
    } else {
      MathSharedStore.reportError(error, stackTrace);
    }
  }

  /**
   * Set error reporting function. By default, DriverStation.reportError() is used.
   *
   * @param func Error reporting function, arguments are error and stackTrace.
   */
  public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
    errorFunc = func;
  }

  /**
   * Generates a trajectory from the given control vectors and config. This method uses clamped
   * cubic splines -- a method in which the exterior control vectors and interior waypoints are
   * provided. The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param initial The initial control vector.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending control vector.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      Spline.ControlVector initial,
      List<Translation2d> interiorWaypoints,
      Spline.ControlVector end,
      TrajectoryConfig config) {
    // Clone the control vectors.
    var newInitial = new Spline.ControlVector(initial.x, initial.y);
    var newEnd = new Spline.ControlVector(end.x, end.y);

    // Change the orientation if reversed.
    if (config.isReversed()) {
      newInitial.x[1] *= -1;
      newInitial.y[1] *= -1;
      newEnd.x[1] *= -1;
      newEnd.y[1] *= -1;
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.getCubicSplinesFromControlVectors(
                  newInitial, interiorWaypoints.toArray(new Translation2d[0]), newEnd));
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(kFlip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses clamped cubic
   * splines -- a method in which the initial pose, final pose, and interior waypoints are provided.
   * The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
    var controlVectors =
        SplineHelper.getCubicControlVectorsFromWaypoints(
            start, interiorWaypoints.toArray(new Translation2d[0]), end);

    // Return the generated trajectory.
    return generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config);
  }

/** A pair of the {@link Landmark#key}, a floating point number indicating position with 
 * respect to the waypoints, and the {@link Landmark#state} of the trajectory at that position.
 * <p> The state may be left null at initialization, since it will be assigned when the
 * trajectory is generated.
 */
static final public class Landmark {
  /** sum of the index for the preceding waypoint (the starting point is 0) and
   * a number between 0.0 and 1.0 estimating position between the preceding (0.0)
   * and following (1.0) waypoints
   */
  public final double key;
  /** state at this positon, particularly including {@code state.timeSeconds},
   * the time the state will be reached.
   * @see Trajectory.State
   */
  public Trajectory.State state;
  /** Creates a {@link Landmark} to pass to {@link TrajectoryGenerator#generateTrajectory(Pose2d, List, Pose2d, TrajectoryConfig, Collection)}
   * <p> {@link Landmark#state} will start out as null.
   */
  public Landmark(double key) {
    this.key = key;
  } 
}
  /**
   * Generates a trajectory from the given control vectors and config. This method uses clamped
   * cubic splines -- a method in which the exterior control vectors and interior waypoints are
   * provided. The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param initial The initial control vector.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending control vector.
   * @param config The configuration for the trajectory.
   * @param landmarks A {@link Collection} of {@link Landmark}s to report about from 
   *  within the Trajectory
   * @return The generated trajectory.
   * @throws IndexOutOfBoundsException if any member of landmarks has a {@code (key < 0 || > interiorWaypoints.size() + 1)}
   *  i. e. the landmark is not on the trajectory
   */
  public static Trajectory generateTrajectory(
      Spline.ControlVector initial,
      List<Translation2d> interiorWaypoints,
      Spline.ControlVector end,
      TrajectoryConfig config,
      Collection<Landmark> landmarks) {

    // Sort the landmarks
    var landmarkArray =  landmarks.toArray(new Landmark[0]);
    Arrays.sort(landmarkArray, (a, b) -> Double.compare(a.key, b.key));
    // Prepare an array for parametrization
    var length = landmarkArray.length;
    var landmarkInfo = new SplineParameterizer.LandmarkInfo[length];
    for (int ix = 0; ix < length; ++ix) {
      landmarkInfo[ix] = new SplineParameterizer.LandmarkInfo(landmarkArray[ix].key);
    }

    // Clone the control vectors.
    var newInitial = new Spline.ControlVector(initial.x, initial.y);
    var newEnd = new Spline.ControlVector(end.x, end.y);

    // Change the orientation if reversed.
    if (config.isReversed()) {
      newInitial.x[1] *= -1;
      newInitial.y[1] *= -1;
      newEnd.x[1] *= -1;
      newEnd.y[1] *= -1;
    }

    // Get the spline points
    // and the landmark info
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.getCubicSplinesFromControlVectors(
                  newInitial, interiorWaypoints.toArray(new Translation2d[0]), newEnd),
                  landmarkInfo);
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(kFlip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    var traj = TrajectoryParameterizer.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
    // fill landmarks in. 
    // This will throw an exception (IndexOutOfBoundsException) if the original key is not part of the trajectory 
    for (int ix = 0; ix < length; ++ix) {
      LandmarkInfo info = landmarkInfo[ix];
      double param = info.splineParamOut;
      Trajectory.State state0 = param != 1 ? traj.getStates().get(info.splineIndexOut) : null,
                       state1 = param != 0 ? traj.getStates().get(info.splineIndexOut+1) : null,
                       state;
      state = state0 == null ? state1 :
              state1 == null ? state0 : 
              state0.interpolate(state1, param);
      landmarkArray[ix].state = state;
    }
    return traj;
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses clamped cubic
   * splines -- a method in which the initial pose, final pose, and interior waypoints are provided.
   * The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   * @param config The configuration for the trajectory.
   * @param landmarks A {@link Collection} of {@link Landmark}s to report about from 
   *  within the Trajectory
   * @return The generated trajectory.
   * @throws IndexOutOfBoundsException if any member of landmarks has a {@code (key < 0 || > interiorWaypoints.size() + 1)}
   *  i. e. the landmark is not on the trajectory
   */
  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config,
      Collection<Landmark> landmarks) {
    var controlVectors =
        SplineHelper.getCubicControlVectorsFromWaypoints(
            start, interiorWaypoints.toArray(new Translation2d[0]), end);

    // Return the generated trajectory.
    return generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config, landmarks);
  }

  /**
   * Generates a trajectory from the given quintic control vectors and config. This method uses
   * quintic hermite splines -- therefore, all points must be represented by control vectors.
   * Continuous curvature is guaranteed in this method.
   *
   * @param controlVectors List of quintic control vectors.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      ControlVectorList controlVectors, TrajectoryConfig config) {
    final var newControlVectors = new ArrayList<Spline.ControlVector>(controlVectors.size());

    // Create a new control vector list, flipping the orientation if reversed.
    for (final var vector : controlVectors) {
      var newVector = new Spline.ControlVector(vector.x, vector.y);
      if (config.isReversed()) {
        newVector.x[1] *= -1;
        newVector.y[1] *= -1;
      }
      newControlVectors.add(newVector);
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.getQuinticSplinesFromControlVectors(
                  newControlVectors.toArray(new Spline.ControlVector[] {})));
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(kFlip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
   * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature is
   * guaranteed in this method.
   *
   * @param waypoints List of waypoints..
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
    List<Pose2d> newWaypoints = new ArrayList<>();
    if (config.isReversed()) {
      for (Pose2d originalWaypoint : waypoints) {
        newWaypoints.add(originalWaypoint.plus(kFlip));
      }
    } else {
      newWaypoints.addAll(waypoints);
    }

    // Get the spline points
    List<PoseWithCurvature> points;
    try {
      points =
          splinePointsFromSplines(
              SplineHelper.optimizeCurvature(
                  SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints)));
    } catch (MalformedSplineException ex) {
      reportError(ex.getMessage(), ex.getStackTrace());
      return kDoNothingTrajectory;
    }

    // Change the points back to their original orientation.
    if (config.isReversed()) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(kFlip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    // Generate and return trajectory.
    return TrajectoryParameterizer.timeParameterizeTrajectory(
        points,
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());
  }

  /**
   * Generate spline points from a vector of splines by parameterizing the splines.
   *
   * @param splines The splines to parameterize.
   * @return The spline points for use in time parameterization of a trajectory.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines) {
    // Create the vector of spline points.
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0).get());

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    for (final var spline : splines) {
      var points = SplineParameterizer.parameterize(spline);

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size()));
    }
    return splinePoints;
  }

  /**
   * Generate spline points from a vector of splines by parameterizing the splines.
   *
   * @param splines The splines to parameterize.
   * @param landmarkInfos landmarks to work in
   * @see SplineParameterizer.LandmarkInfo
   * @return The spline points for use in time parameterization of a trajectory.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines, SplineParameterizer.LandmarkInfo[] landmarkInfos) {
    // Create the vector of spline points.
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0).get());
    
    /** to keep track of which spline we're working on */
    int splineIx = 0;
    /** the index of the next landmark to initialize */
    var landmarkIx = new SplineParameterizer.IntRef();

    // Advance landmark index through any negative indices.
    // The only case that won't raise OutOfBounds errors is splineIndexIn = -1, splineParamIn = 1.0
    // which comes from key == 0.
    for(;
    landmarkIx.val < landmarkInfos.length 
          && (landmarkInfos[landmarkIx.val]).splineIndexIn < 0;
          ++ landmarkIx.val); 
          
    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    // initialize landmarks as we come across them
    for (final var spline : splines) {
      var points = SplineParameterizer.parameterize(spline, splineIx++, splinePoints.size() - 2, landmarkInfos, landmarkIx);

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size()));
    }
    return splinePoints;
  }

  /** Control vector list type that works around type erasure signatures. */
  public static class ControlVectorList extends ArrayList<Spline.ControlVector> {
    /** Default constructor. */
    public ControlVectorList() {
      super();
    }

    /**
     * Constructs a ControlVectorList.
     *
     * @param initialCapacity The initial list capacity.
     */
    public ControlVectorList(int initialCapacity) {
      super(initialCapacity);
    }

    /**
     * Constructs a ControlVectorList.
     *
     * @param collection A collection of spline control vectors.
     */
    public ControlVectorList(Collection<? extends Spline.ControlVector> collection) {
      super(collection);
    }
  }
}
