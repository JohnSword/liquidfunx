/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package box2d.collision;

import box2d.collision.DistanceProxy;
import box2d.collision.SimplexCache;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Sweep;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.pooling.IWorldPool;

import haxe.ds.Vector;

enum TOIOutputState {
    UNKNOWN; 
    FAILED; 
    OVERLAPPED;
    TOUCHING; 
    SEPARATED;
  }

/**
 * Class used for computing the time of impact. This class should not be constructed usually, just
 * retrieve from the {@link IWorldPool#getTimeOfImpact()}.
 * 
 * @author daniel
 */
 class TimeOfImpact {
  public static var MAX_ITERATIONS : Int = 20;
  public static var MAX_ROOT_ITERATIONS : Int = 50;

  public static var toiCalls : Int = 0;
  public static var toiIters : Int = 0;
  public static var toiMaxIters : Int = 0;
  public static var toiRootIters : Int = 0;
  public static var toiMaxRootIters : Int = 0;

  // djm pooling
  private var cache : SimplexCache = new SimplexCache();
  private var distanceInput : DistanceInput = new DistanceInput();
  private var xfA : Transform = new Transform();
  private var xfB : Transform = new Transform();
  private var distanceOutput : DistanceOutput = new DistanceOutput();
  private var fcn : SeparationFunction = new SeparationFunction();
  private var indexes : Vector<Int>;
  private var sweepA : Sweep = new Sweep();
  private var sweepB : Sweep = new Sweep();

  private var pool : IWorldPool;

  public function new(argPool : IWorldPool) {
    pool = argPool;
  }

  /**
   * Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
   * between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
   * non-tunneling collision. If you change the time interval, you should call this function again.
   * Note: use Distance to compute the contact point and normal at the time of impact.
   * 
   * @param output
   * @param input
   */
  public function timeOfImpact(output : TOIOutput, input : TOIInput) : Void {
    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.

    ++toiCalls;

    output.state = TOIOutputState.UNKNOWN;
    output.t = input.tMax;

    var proxyA : DistanceProxy = input.proxyA;
    var proxyB : DistanceProxy = input.proxyB;

    sweepA.set(input.sweepA);
    sweepB.set(input.sweepB);

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.normalize();
    sweepB.normalize();

    var tMax : Float = input.tMax;

    var totalRadius : Float = proxyA.m_radius + proxyB.m_radius;
    // djm: whats with all these constants?
    var target : Float = MathUtils.max(Settings.linearSlop, totalRadius - 3.0 * Settings.linearSlop);
    var tolerance : Float = 0.25 * Settings.linearSlop;


    var t1 : Float = 0;
    var iter : Int = 0;

    cache.count = 0;
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    while (true) {
      sweepA.getTransform(xfA, t1);
      sweepB.getTransform(xfB, t1);
      // System.out.printf("sweepA: %f, %f, sweepB: %f, %f\n",
      // sweepA.c.x, sweepA.c.y, sweepB.c.x, sweepB.c.y);
      // Get the distance between shapes. We can also use the results
      // to get a separating axis
      distanceInput.transformA = xfA;
      distanceInput.transformB = xfB;
      pool.getDistance().distance(distanceOutput, cache, distanceInput);

      // System.out.printf("Dist: %f at points %f, %f and %f, %f.  %d iterations\n",
      // distanceOutput.distance, distanceOutput.pointA.x, distanceOutput.pointA.y,
      // distanceOutput.pointB.x, distanceOutput.pointB.y,
      // distanceOutput.iterations);

      // If the shapes are overlapped, we give up on continuous collision.
      if (distanceOutput.distance <= 0) {
        // Failure!
        output.state = TOIOutputState.OVERLAPPED;
        output.t = 0;
        break;
      }

      if (distanceOutput.distance < target + tolerance) {
        // Victory!
        output.state = TOIOutputState.TOUCHING;
        output.t = t1;
        break;
      }

      // Initialize the separating axis.
      fcn.initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

      // Compute the TOI on the separating axis. We do this by successively
      // resolving the deepest point. This loop is bounded by the number of
      // vertices.
      var done : Bool = false;
      var t2 : Float = tMax;
      var pushBackIter : Int = 0;
      while (true) {

        // Find the deepest point at t2. Store the witness point indices.
        var s2 : Float = fcn.findMinSeparation(indexes, t2);
        // System.out.printf("s2: %f\n", s2);
        // Is the final configuration separated?
        if (s2 > target + tolerance) {
          // Victory!
          output.state = TOIOutputState.SEPARATED;
          output.t = tMax;
          done = true;
          break;
        }

        // Has the separation reached tolerance?
        if (s2 > target - tolerance) {
          // Advance the sweeps
          t1 = t2;
          break;
        }

        // Compute the initial separation of the witness points.
        var s1 : Float = fcn.evaluate(indexes[0], indexes[1], t1);
        // Check for initial overlap. This might happen if the root finder
        // runs out of iterations.
        // System.out.printf("s1: %f, target: %f, tolerance: %f\n", s1, target,
        // tolerance);
        if (s1 < target - tolerance) {
          output.state = TOIOutputState.FAILED;
          output.t = t1;
          done = true;
          break;
        }

        // Check for touching
        if (s1 <= target + tolerance) {
          // Victory! t1 should hold the TOI (could be 0.0).
          output.state = TOIOutputState.TOUCHING;
          output.t = t1;
          done = true;
          break;
        }

        // Compute 1D root of: f(x) - target = 0
        var rootIterCount : Int = 0;
        var a1 : Float = t1, a2 = t2;
        while (true) {
          // Use a mix of the secant rule and bisection.
          var t : Float;
          if ((rootIterCount & 1) == 1) {
            // Secant rule to improve convergence.
            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
          } else {
            // Bisection to guarantee progress.
            t = 0.5 * (a1 + a2);
          }

          ++rootIterCount;
          ++toiRootIters;

          var s : Float = fcn.evaluate(indexes[0], indexes[1], t);

          if (MathUtils.abs(s - target) < tolerance) {
            // t2 holds a tentative value for t1
            t2 = t;
            break;
          }

          // Ensure we continue to bracket the root.
          if (s > target) {
            a1 = t;
            s1 = s;
          } else {
            a2 = t;
            s2 = s;
          }

          if (rootIterCount == MAX_ROOT_ITERATIONS) {
            break;
          }
        }

        toiMaxRootIters = MathUtils.max(toiMaxRootIters, rootIterCount);

        ++pushBackIter;

        if (pushBackIter == Settings.maxPolygonVertices || rootIterCount == MAX_ROOT_ITERATIONS) {
          break;
        }
      }

      ++iter;
      ++toiIters;

      if (done) {
        // System.out.println("done");
        break;
      }

      if (iter == MAX_ITERATIONS) {
        // System.out.println("failed, root finder stuck");
        // Root finder got stuck. Semi-victory.
        output.state = TOIOutputState.FAILED;
        output.t = t1;
        break;
      }
    }

    // System.out.printf("final sweeps: %f, %f, %f; %f, %f, %f", input.s)
    toiMaxIters = MathUtils.max(toiMaxIters, iter);
  }
}
