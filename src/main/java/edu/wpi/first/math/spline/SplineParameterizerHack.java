// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Hacked by Philp Harrington

/*
 * MIT License
 *
 * Copyright (c) 2018 Team 254
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package edu.wpi.first.math.spline;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/** Class used to parameterize a spline by its arc length.
 * <p> members added by Harrington:
 * <p>{@link LandmarkInfo}, {@link IntRef},
 * <p>{@link SplineParameterizerHack#parameterize(Spline, int, int, LandmarkInfo[], IntRef)},
 * <p>and {@link SplineParameterizerHack#parameterize(Spline, int, int, LandmarkInfo[], IntRef, double, double)}
 */
public final class SplineParameterizerHack {
  private static final double kMaxDx = 0.127;
  private static final double kMaxDy = 0.00127;
  private static final double kMaxDtheta = 0.0872;

  private static final String kMalformedSplineExceptionMsg =
      "Could not parameterize a malformed spline. This means that you probably had two or more "
          + "adjacent waypoints that were very close together with headings in opposing "
          + "directions.";

  /**
   * A malformed spline does not actually explode the LIFO stack size. Instead, the stack size stays
   * at a relatively small number (e.g. 30) and never decreases. Because of this, we must count
   * iterations. Even long, complex paths don't usually go over 300 iterations, so hitting this
   * maximum should definitely indicate something has gone wrong.
   */
  private static final int kMaxIterations = 5000;

  private static class StackContents {
    final double t1;
    final double t0;

    StackContents(double t0, double t1) {
      this.t0 = t0;
      this.t1 = t1;
    }
  }

  /** Exception for malformed splines. */
  public static final class MalformedSplineException extends RuntimeException {
    /**
     * Create a new exception with the given message.
     *
     * @param message the message to pass with the exception
     */
    private MalformedSplineException(String message) {
      super(message);
    }
  }

  /** Private constructor because this is a utility class. */
  private SplineParameterizerHack() {}

  /**
   * Parametrizes the spline. This method breaks up the spline into various arcs until their dx, dy,
   * and dtheta are within specific tolerances.
   *
   * @param spline The spline to parameterize.
   * @return A list of poses and curvatures that represents various points on the spline.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> parameterize(Spline spline) {
    return parameterize(spline, 0.0, 1.0);
  }

  /**
   * Parametrizes the spline. This method breaks up the spline into various arcs until their dx, dy,
   * and dtheta are within specific tolerances.
   *
   * @param spline The spline to parameterize.
   * @param t0 Starting internal spline parameter. It is recommended to use 0.0.
   * @param t1 Ending internal spline parameter. It is recommended to use 1.0.
   * @return A list of poses and curvatures that represents various points on the spline.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> parameterize(Spline spline, double t0, double t1) {
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // The parameterization does not add the initial point. Let's add that.
    splinePoints.add(spline.getPoint(t0).get());

    // We use an "explicit stack" to simulate recursion, instead of a recursive function call
    // This give us greater control, instead of a stack overflow
    var stack = new ArrayDeque<StackContents>();
    stack.push(new StackContents(t0, t1));

    int iterations = 0;

    while (!stack.isEmpty()) {
      var current = stack.removeFirst();

      var start = spline.getPoint(current.t0);
      if (!start.isPresent()) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }

      var end = spline.getPoint(current.t1);
      if (!end.isPresent()) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }

      final var twist = start.get().poseMeters.log(end.get().poseMeters);
      if (Math.abs(twist.dy) > kMaxDy
          || Math.abs(twist.dx) > kMaxDx
          || Math.abs(twist.dtheta) > kMaxDtheta) {
        stack.addFirst(new StackContents((current.t0 + current.t1) / 2, current.t1));
        stack.addFirst(new StackContents(current.t0, (current.t0 + current.t1) / 2));
      } else {
        splinePoints.add(end.get());
      }

      iterations++;
      if (iterations >= kMaxIterations) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }
    }

    return splinePoints;
  }

  /** Information about a position on a trajectory,
   * both before and after parametrization.
   */
  public static final class LandmarkInfo {
    /** the spline the landmark is in before */
    public final int splineIndexIn; 
    /** the displacement ([0.0, 1.0]) within the spline before */
    final double splineParamIn;
    /** the spline the landmark is in after */
    public int splineIndexOut = -1;
    /** the displacement ([0.0, 1.0]) within the spline after */
    public double splineParamOut;

    public LandmarkInfo(double key) {
      splineIndexIn = (int)(Math.ceil(key)) - 1;
      splineParamIn = key - splineIndexIn;
      splineParamOut = splineParamIn;
    }
    void divide() {
      splineParamOut *= 2;
      if (splineParamOut > 1) splineParamOut -= 1;
    }
  }

  public static class IntRef {
    public int val;
  }

  /**
   * Parametrizes the spline. This method breaks up the spline into various arcs until their dx, dy,
   * and dtheta are within specific tolerances.
   *
   * @param spline The spline to parameterize.
   * @param splineIn which  spline is this
   * @param splinesOut how many spline have already been added to the list
   * @param landmarks an array of positions along the trajectory, see {@link LandmarkInfo}
   * @param landmarkIx where to start in landmarks; it's an IntRef to propagate changes 
   * @return A list of poses and curvatures that represents various points on the spline.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> parameterize(Spline spline, int splineIn, int splinesOut,
       final LandmarkInfo[] landmarks, final IntRef landmarkIx) {
    return parameterize(spline, splineIn, splinesOut,
        landmarks, landmarkIx, 0.0, 1.0);
  }

  /**
   * Parametrizes the spline. This method breaks up the spline into various arcs until their dx, dy,
   * and dtheta are within specific tolerances.
   * The landmarks will have their output information filled in.
   *
   * @param spline The spline to parameterize.
   * @param splineIn which  spline is this
   * @param splinesOut how many spline have already been added to the list
   * @param landmarks an array of positions along the trajectory, see {@link LandmarkInfo}
   * @param landmarkIx where to start in landmarks; it's an {@link IntRef} to propagate changes
   * @param t0 Starting internal spline parameter. It is recommended to use 0.0.
   * @param t1 Ending internal spline parameter. It is recommended to use 1.0.
   * @return A list of poses and curvatures that represents various points on the spline.
   * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
   *     with approximately opposing headings)
   */
  public static List<PoseWithCurvature> parameterize(
       Spline spline, int splineIn, int splinesOut,
       final LandmarkInfo[] landmarks, final IntRef landmarkIx, double t0, double t1) {
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // The parameterization does not add the initial point. Let's add that.
    splinePoints.add(spline.getPoint(t0).get());

    // We use an "explicit stack" to simulate recursion, instead of a recursive function call
    // This give us greater control, instead of a stack overflow
    var stack = new ArrayDeque<StackContents>();
    stack.push(new StackContents(t0, t1));

    int iterations = 0;

    while (!stack.isEmpty()) {
      var current = stack.removeFirst();

      var start = spline.getPoint(current.t0);
      if (!start.isPresent()) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }

      var end = spline.getPoint(current.t1);
      if (!end.isPresent()) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }

      final var twist = start.get().poseMeters.log(end.get().poseMeters);
        LandmarkInfo l;
      if (Math.abs(twist.dy) > kMaxDy
          || Math.abs(twist.dx) > kMaxDx
          || Math.abs(twist.dtheta) > kMaxDtheta) {
        for (
          int ix = landmarkIx.val; 
          ix < landmarks.length 
          && (l = landmarks[ix]).splineIndexIn == splineIn 
          && l.splineParamIn <= current.t1;
          ++ ix
          ) 
          l.divide();
        stack.addFirst(new StackContents((current.t0 + current.t1) / 2, current.t1));
        stack.addFirst(new StackContents(current.t0, (current.t0 + current.t1) / 2));
      } else {
        for (
          ; 
          landmarkIx.val < landmarks.length 
          && (l = landmarks[landmarkIx.val]).splineIndexIn == splineIn 
          && l.splineParamIn <= current.t1;
          ++ landmarkIx.val) 
          l.splineIndexOut = splinePoints.size() + splinesOut;
        splinePoints.add(end.get());
      }

      iterations++;
      if (iterations >= kMaxIterations) {
        throw new MalformedSplineException(kMalformedSplineExceptionMsg);
      }
    }

    return splinePoints;
  }
}

