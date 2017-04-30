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

import box2d.collision.shapes.ChainShape;
import box2d.collision.shapes.CircleShape;
import box2d.collision.shapes.EdgeShape;
import box2d.collision.shapes.PolygonShape;
import box2d.collision.shapes.Shape;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.common.Transform;

// updated to rev 100
/**
 * This is non-static for faster pooling. To get an instance, use the {@link SingletonPool}, don't
 * construct a distance object.
 * 
 * @author Daniel Murphy
 */
 class Distance {
  public static var MAX_ITERS : Int = 20;

  public static var GJK_CALLS : Int = 0;
  public static var GJK_ITERS : Int = 0;
  public static var GJK_MAX_ITERS : Int = 20;

  private var simplex : Simplex = new Simplex();
  private var saveA : Array<Int> = new Array<Int>();
  private var saveB : Array<Int> = new Array<Int>();
  private var closestPoint : Vec2 = new Vec2();
  private var d : Vec2 = new Vec2();
  private var temp : Vec2 = new Vec2();
  private var normal : Vec2 = new Vec2();

  public function new() {}

  /**
   * Compute the closest points between two shapes. Supports any combination of: CircleShape and
   * PolygonShape. The simplex cache is input/output. On the first call set SimplexCache.count to
   * zero.
   * 
   * @param output
   * @param cache
   * @param input
   */
  public function distance(output : DistanceOutput, cache : SimplexCache, input : DistanceInput) : Void {
    GJK_CALLS++;

    var proxyA : DistanceProxy = input.proxyA;
    var proxyB : DistanceProxy = input.proxyB;

    var transformA : Transform = input.transformA;
    var transformB : Transform = input.transformB;

    // Initialize the simplex.
    simplex.readCache(cache, proxyA, transformA, proxyB, transformB);

    // Get simplex vertices as an array.
    var vertices : Array<SimplexVertex> = simplex.vertices;

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    // (pooled above)
    var saveCount : Int = 0;

    simplex.getClosestPoint(closestPoint);
    var distanceSqr1 : Float = closestPoint.lengthSquared();
    var distanceSqr2 : Float = distanceSqr1;

    // Main iteration loop
    var iter : Int = 0;
    while (iter < MAX_ITERS) {

      // Copy simplex so we can identify duplicates.
      saveCount = simplex.m_count;
      for(i in 0 ... saveCount) {
        saveA[i] = vertices[i].indexA;
        saveB[i] = vertices[i].indexB;
      }

      switch (simplex.m_count) {
      case 1:
        break;
      case 2:
        simplex.solve2();
        break;
      case 3:
        simplex.solve3();
        break;
      default:
      }

      // If we have 3 points, then the origin is in the corresponding triangle.
      if (simplex.m_count == 3) {
        break;
      }

      // Compute closest point.
      simplex.getClosestPoint(closestPoint);
      distanceSqr2 = closestPoint.lengthSquared();

      // ensure progress
      if (distanceSqr2 >= distanceSqr1) {
        // break;
      }
      distanceSqr1 = distanceSqr2;

      // get search direction;
      simplex.getSearchDirection(d);

      // Ensure the search direction is numerically fit.
      if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON) {
        // The origin is probably contained by a line segment
        // or triangle. Thus the shapes are overlapped.

        // We can't return zero here even though there may be overlap.
        // In case the simplex is a point, segment, or triangle it is difficult
        // to determine if the origin is contained in the CSO or very close to it.
        break;
      }
      /*
       * SimplexVertex* vertex = vertices + simplex.m_count; vertex.indexA =
       * proxyA.GetSupport(MulT(transformA.R, -d)); vertex.wA = Mul(transformA,
       * proxyA.GetVertex(vertex.indexA)); Vec2 wBLocal; vertex.indexB =
       * proxyB.GetSupport(MulT(transformB.R, d)); vertex.wB = Mul(transformB,
       * proxyB.GetVertex(vertex.indexB)); vertex.w = vertex.wB - vertex.wA;
       */

      // Compute a tentative new simplex vertex using support points.
      var vertex : SimplexVertex = vertices[simplex.m_count];

      Rot.mulTransUnsafe2(transformA.q, d.negateLocal(), temp);
      vertex.indexA = proxyA.getSupport(temp);
      Transform.mulToOutUnsafe(transformA, proxyA.getVertex(vertex.indexA), vertex.wA);
      // Vec2 wBLocal;
      Rot.mulTransUnsafe2(transformB.q, d.negateLocal(), temp);
      vertex.indexB = proxyB.getSupport(temp);
      Transform.mulToOutUnsafe(transformB, proxyB.getVertex(vertex.indexB), vertex.wB);
      vertex.w.setVec(vertex.wB).subLocal(vertex.wA);

      // Iteration count is equated to the number of support point calls.
      ++iter;
      ++GJK_ITERS;

      // Check for duplicate support points. This is the main termination criteria.
      var duplicate : Bool = false;
      var i:Int = 0;
      while(i < saveCount) {
      // for (int i = 0; i < saveCount; ++i) {
        if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
          duplicate = true;
          break;
        }
        ++i;
      }

      // If we found a duplicate support point we must exit to avoid cycling.
      if (duplicate) {
        break;
      }

      // New vertex is ok and needed.
      ++simplex.m_count;
    }

    GJK_MAX_ITERS = MathUtils.max(GJK_MAX_ITERS, iter);

    // Prepare output.
    simplex.getWitnessPoints(output.pointA, output.pointB);
    output.distance = MathUtils.distance(output.pointA, output.pointB);
    output.iterations = iter;

    // Cache the simplex.
    simplex.writeCache(cache);

    // Apply radii if requested.
    if (input.useRadii) {
      var rA : Float = proxyA.m_radius;
      var rB : Float = proxyB.m_radius;

      if (output.distance > rA + rB && output.distance > Settings.EPSILON) {
        // Shapes are still no overlapped.
        // Move the witness points to the outer surface.
        output.distance -= rA + rB;
        normal.setVec(output.pointB).subLocal(output.pointA);
        normal.normalize();
        temp.setVec(normal).mulLocal(rA);
        output.pointA.addLocalVec(temp);
        temp.setVec(normal).mulLocal(rB);
        output.pointB.subLocal(temp);
      } else {
        // Shapes are overlapped when radii are considered.
        // Move the witness points to the middle.
        // Vec2 p = 0.5f * (output.pointA + output.pointB);
        output.pointA.addLocalVec(output.pointB).mulLocal(.5);
        output.pointB.setVec(output.pointA);
        output.distance = 0.0;
      }
    }
  }
}

