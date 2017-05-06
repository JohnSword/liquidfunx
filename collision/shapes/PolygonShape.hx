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
package box2d.collision.shapes;

import box2d.collision.AABB;
import box2d.collision.RayCastInput;
import box2d.collision.RayCastOutput;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.pooling.arrays.IntArray;
import box2d.pooling.arrays.Vec2Array;

import haxe.ds.Vector;

/**
 * A convex polygon shape. Polygons have a maximum number of vertices equal to _maxPolygonVertices.
 * In most cases you should not need many vertices for a convex polygon.
 */
 class PolygonShape extends Shape {
  /** Dump lots of debug information. */
  private static var m_debug : Bool = false;

  /**
   * Local position of the shape centroid in parent body frame.
   */
  public var m_centroid : Vec2 = new Vec2();

  /**
   * The vertices of the shape. Note: use getVertexCount(), not m_vertices.length, to get number of
   * active vertices.
   */
  public var m_vertices : Vector<Vec2>;

  /**
   * The normals of the shape. Note: use getVertexCount(), not m_normals.length, to get number of
   * active normals.
   */
  public var m_normals: Vector<Vec2>;

  /**
   * Number of active vertices in the shape.
   */
  public var m_count : Int = 0;

  // pooling
  private var pool1 : Vec2 = new Vec2();
  private var pool2 : Vec2 = new Vec2();
  private var pool3 : Vec2 = new Vec2();
  private var pool4 : Vec2 = new Vec2();
  private var poolt1 : Transform = new Transform();

  public function new() {
    super(ShapeType.POLYGON);

    m_count = 0;
    m_vertices = new Vector<Vec2>(Settings.maxPolygonVertices);
    for(i in 0 ... m_vertices.length) {
      m_vertices[i] = new Vec2();
    }
    m_normals = new Vector<Vec2>(Settings.maxPolygonVertices);
    for(i in 0 ... m_normals.length) {
      m_normals[i] = new Vec2();
    }
    setRadius(Settings.polygonRadius);
    m_centroid.setZero();
  }

  override public function clone() : Shape {
    var shape : PolygonShape = new PolygonShape();
    shape.m_centroid.setVec(this.m_centroid);
    for(i in 0 ... shape.m_normals.length) {
      shape.m_normals[i].setVec(m_normals[i]);
      shape.m_vertices[i].setVec(m_vertices[i]);
    }
    shape.setRadius(this.getRadius());
    shape.m_count = this.m_count;
    return shape;
  }

  /**
   * Create a convex hull from the given array of points. The count must be in the range [3,
   * Settings.maxPolygonVertices].
   * 
   * @warning the points may be re-ordered, even if they form a convex polygon.
   * @warning collinear points are removed.
   */
  public function set(vertices : Array<Vec2>, count : Int) : Void {
    set2(vertices, count, null, null);
  }

  /**
   * Create a convex hull from the given array of points. The count must be in the range [3,
   * Settings.maxPolygonVertices]. This method takes an arraypool for pooling.
   * 
   * @warning the points may be re-ordered, even if they form a convex polygon.
   * @warning collinear points are removed.
   */
  public function set2(verts : Array<Vec2>, num : Int, vecPool : Vec2Array, intPool : IntArray) : Void {
    if (num < 3) {
      setAsBox(1.0, 1.0);
      return;
    }

    var n : Int = MathUtils.min(num, Settings.maxPolygonVertices);

    // Perform welding and copy vertices into local buffer.
    var ps : Vector<Vec2> =
        (vecPool != null)
            ? vecPool.get(Settings.maxPolygonVertices)
            : new Vector<Vec2>(Settings.maxPolygonVertices);
    var tempCount : Int = 0;
    for (i in 0 ... n) {
    // for (int i = 0; i < n; ++i) {
      var v : Vec2 = verts[i];
      var unique : Bool = true;
      for (j in 0 ... tempCount) {
      // for (int j = 0; j < tempCount; ++j) {
        if (MathUtils.distanceSquared(v, ps[j]) < 0.5 * Settings.linearSlop) {
          unique = false;
          break;
        }
      }

      if (unique) {
        ps[tempCount++] = v;
      }
    }

    n = tempCount;
    if (n < 3) {
      // Polygon is degenerate.
      setAsBox(1.0, 1.0);
      return;
    }

    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

    // Find the right most point on the hull
    var i0 : Int = 0;
    var x0 : Float = ps[0].x;
    
    for(i in 1 ... n) {
    // for (int i = 1; i < n; ++i) {
      var x : Float = ps[i].x;
      if (x > x0 || (x == x0 && ps[i].y < ps[i0].y)) {
        i0 = i;
        x0 = x;
      }
    }

    var hull : Vector<Int> =
        (intPool != null)
            ? intPool.get(Settings.maxPolygonVertices)
            : new Vector<Int>(Settings.maxPolygonVertices);
    var m : Int = 0;
    var ih : Int = i0;

    while (true) {
      hull[m] = ih;

      var ie : Int = 0;
      for (j in 1 ... n) {
      // for (int j = 1; j < n; ++j) {
        if (ie == ih) {
          ie = j;
          continue;
        }

        var r : Vec2 = pool1.setVec(ps[ie]).subLocal(ps[hull[m]]);
        var v : Vec2 = pool2.setVec(ps[j]).subLocal(ps[hull[m]]);
        var c : Float = Vec2.crossVec(r, v);
        if (c < 0.0) {
          ie = j;
        }

        // Collinearity check
        if (c == 0.0 && v.lengthSquared() > r.lengthSquared()) {
          ie = j;
        }

      }

      ++m;
      ih = ie;

      if (ie == i0) {
        break;
      }
    }

    this.m_count = m;

    // Copy vertices.
    for(i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      if (m_vertices[i] == null) {
        m_vertices[i] = new Vec2();
      }
      m_vertices[i].setVec(ps[hull[i]]);
    }

    var edge : Vec2 = pool1;

    // Compute normals. Ensure the edges have non-zero length.
    for(i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var i1 : Int = i;
      var i2 : Int = i + 1 < m_count ? i + 1 : 0;
      edge.setVec(m_vertices[i2]).subLocal(m_vertices[i1]);

      Vec2.crossToOutUnsafe(edge, 1, m_normals[i]);
      m_normals[i].normalize();
    }

    // Compute the polygon centroid.
    computeCentroidToOut(m_vertices, m_count, m_centroid);
  }

  /**
   * Build vertices to represent an axis-aligned box.
   * 
   * @param hx the half-width.
   * @param hy the half-height.
   */
  public function setAsBox(hx : Float, hy : Float) : Void {
    m_count = 4;
    m_vertices[0].set(-hx, -hy);
    m_vertices[1].set(hx, -hy);
    m_vertices[2].set(hx, hy);
    m_vertices[3].set(-hx, hy);
    m_normals[0].set(0.0, -1.0);
    m_normals[1].set(1.0, 0.0);
    m_normals[2].set(0.0, 1.0);
    m_normals[3].set(-1.0, 0.0);
    m_centroid.setZero();
  }

  /**
   * Build vertices to represent an oriented box.
   * 
   * @param hx the half-width.
   * @param hy the half-height.
   * @param center the center of the box in local coordinates.
   * @param angle the rotation of the box in local coordinates.
   */
  public function setAsBox2(hx : Float, hy : Float, center : Vec2, angle : Float) : Void {
    m_count = 4;
    m_vertices[0].set(-hx, -hy);
    m_vertices[1].set(hx, -hy);
    m_vertices[2].set(hx, hy);
    m_vertices[3].set(-hx, hy);
    m_normals[0].set(0.0, -1.0);
    m_normals[1].set(1.0, 0.0);
    m_normals[2].set(0.0, 1.0);
    m_normals[3].set(-1.0, 0.0);
    m_centroid.setVec(center);

    var xf : Transform = poolt1;
    xf.p.setVec(center);
    xf.q.set(angle);

    // Transform vertices and normals.
    for(i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      Transform.mulToOut(xf, m_vertices[i], m_vertices[i]);
      Rot.mulToOut(xf.q, m_normals[i], m_normals[i]);
    }
  }

  override public function getChildCount() : Int {
    return 1;
  }

  override public function testPoint(xf : Transform, p : Vec2) : Bool {
    var tempx : Float, tempy : Float;
    var xfq : Rot = xf.q;

    tempx = p.x - xf.p.x;
    tempy = p.y - xf.p.y;
    var pLocalx : Float = xfq.c * tempx + xfq.s * tempy;
    var pLocaly : Float = -xfq.s * tempx + xfq.c * tempy;

    if (m_debug) {
      trace("--testPoint debug--");
      trace("Vertices: ");
      // System.out.println("--testPoint debug--");
      // System.out.println("Vertices: ");
      var i : Int = 0;
      while (i < m_count) {
      // for (int i = 0; i < m_count; ++i) {
        // System.out.println(m_vertices[i]);
        trace(m_vertices[i]);
        ++i;
      }
      // System.out.println("pLocal: " + pLocalx + ", " + pLocaly);
    }

    for(i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var vertex : Vec2 = m_vertices[i];
      var normal : Vec2 = m_normals[i];
      tempx = pLocalx - vertex.x;
      tempy = pLocaly - vertex.y;
      var dot : Float = normal.x * tempx + normal.y * tempy;
      if (dot > 0.0) {
        return false;
      }
    }

    return true;
  }

  override public function computeAABB(aabb : AABB, xf : Transform, childIndex : Int) : Void {
    var lower : Vec2 = aabb.lowerBound;
    var upper : Vec2 = aabb.upperBound;
    var v1 : Vec2 = m_vertices[0];
    var xfqc : Float = xf.q.c;
    var xfqs : Float = xf.q.s;
    var xfpx : Float = xf.p.x;
    var xfpy : Float = xf.p.y;
    lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx;
    lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy;
    upper.x = lower.x;
    upper.y = lower.y;

    for(i in 1 ... m_count) {
    // for (int i = 1; i < m_count; ++i) {
      var v2 : Vec2 = m_vertices[i];
      var vx : Float = (xfqc * v2.x - xfqs * v2.y) + xfpx;
      var vy : Float = (xfqs * v2.x + xfqc * v2.y) + xfpy;
      lower.x = lower.x < vx ? lower.x : vx;
      lower.y = lower.y < vy ? lower.y : vy;
      upper.x = upper.x > vx ? upper.x : vx;
      upper.y = upper.y > vy ? upper.y : vy;
    }

    lower.x -= m_radius;
    lower.y -= m_radius;
    upper.x += m_radius;
    upper.y += m_radius;
  }

  /**
   * Get the vertex count.
   * 
   * @return
   */
  public function getVertexCount() : Int {
    return m_count;
  }

  /**
   * Get a vertex by index.
   * 
   * @param index
   * @return
   */
  public function getVertex(index : Int) : Vec2 {
    return m_vertices[index];
  }

  override public function computeDistanceToOut(xf : Transform, p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    var xfqc : Float = xf.q.c;
    var xfqs : Float = xf.q.s;
    var tx : Float = p.x - xf.p.x;
    var ty : Float = p.y - xf.p.y;
    var pLocalx : Float = xfqc * tx + xfqs * ty;
    var pLocaly : Float = -xfqs * tx + xfqc * ty;

    // var maxDistance : Float = -Float.MAX_VALUE;
    var maxDistance : Float = -MathUtils.MAX_VALUE;
    var normalForMaxDistanceX : Float = pLocalx;
    var normalForMaxDistanceY : Float = pLocaly;

    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var vertex : Vec2 = m_vertices[i];
      var normal : Vec2 = m_normals[i];
      tx = pLocalx - vertex.x;
      ty = pLocaly - vertex.y;
      var dot : Float = normal.x * tx + normal.y * ty;
      if (dot > maxDistance) {
        maxDistance = dot;
        normalForMaxDistanceX = normal.x;
        normalForMaxDistanceY = normal.y;
      }
    }

    var distance : Float;
    if (maxDistance > 0) {
      var minDistanceX : Float = normalForMaxDistanceX;
      var minDistanceY : Float = normalForMaxDistanceY;
      var minDistance2 : Float = maxDistance * maxDistance;
      for (i in 0 ... m_count) {
      // for (int i = 0; i < m_count; ++i) {
        var vertex : Vec2 = m_vertices[i];
        var distanceVecX : Float = pLocalx - vertex.x;
        var distanceVecY : Float = pLocaly - vertex.y;
        var distance2 : Float = (distanceVecX * distanceVecX + distanceVecY * distanceVecY);
        if (minDistance2 > distance2) {
          minDistanceX = distanceVecX;
          minDistanceY = distanceVecY;
          minDistance2 = distance2;
        }
      }
      distance = MathUtils.sqrt(minDistance2);
      normalOut.x = xfqc * minDistanceX - xfqs * minDistanceY;
      normalOut.y = xfqs * minDistanceX + xfqc * minDistanceY;
      normalOut.normalize();
    } else {
      distance = maxDistance;
      normalOut.x = xfqc * normalForMaxDistanceX - xfqs * normalForMaxDistanceY;
      normalOut.y = xfqs * normalForMaxDistanceX + xfqc * normalForMaxDistanceY;
    }

    return distance;
  }

  public override function raycast(output : RayCastOutput, input : RayCastInput, xf : Transform, childIndex : Int) : Bool {
    var xfqc : Float = xf.q.c;
    var xfqs : Float = xf.q.s;
    var xfp : Vec2 = xf.p;
    var tempx : Float, tempy : Float;
    // b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
    // b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
    tempx = input.p1.x - xfp.x;
    tempy = input.p1.y - xfp.y;
    var p1x : Float = xfqc * tempx + xfqs * tempy;
    var p1y : Float = -xfqs * tempx + xfqc * tempy;

    tempx = input.p2.x - xfp.x;
    tempy = input.p2.y - xfp.y;
    var p2x : Float = xfqc * tempx + xfqs * tempy;
    var p2y : Float = -xfqs * tempx + xfqc * tempy;

    var dx : Float = p2x - p1x;
    var dy : Float = p2y - p1y;

    var lower : Float = 0, upper = input.maxFraction;

    var index : Int = -1;

    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var normal : Vec2 = m_normals[i];
      var vertex : Vec2 = m_vertices[i];
      // p = p1 + a * d
      // dot(normal, p - v) = 0
      // dot(normal, p1 - v) + a * dot(normal, d) = 0
      var tempxn : Float = vertex.x - p1x;
      var tempyn : Float = vertex.y - p1y;
      var numerator : Float = normal.x * tempxn + normal.y * tempyn;
      var denominator : Float = normal.x * dx + normal.y * dy;

      if (denominator == 0.0) {
        if (numerator < 0.0) {
          return false;
        }
      } else {
        // Note: we want this predicate without division:
        // lower < numerator / denominator, where denominator < 0
        // Since denominator < 0, we have to flip the inequality:
        // lower < numerator / denominator <==> denominator * lower >
        // numerator.
        if (denominator < 0.0 && numerator < lower * denominator) {
          // Increase lower.
          // The segment enters this half-space.
          lower = numerator / denominator;
          index = i;
        } else if (denominator > 0.0 && numerator < upper * denominator) {
          // Decrease upper.
          // The segment exits this half-space.
          upper = numerator / denominator;
        }
      }

      if (upper < lower) {
        return false;
      }

    }


    if (index >= 0) {
      output.fraction = lower;
      // normal = Mul(xf.R, m_normals[index]);
      var normal : Vec2 = m_normals[index];
      var out : Vec2 = output.normal;
      out.x = xfqc * normal.x - xfqs * normal.y;
      out.y = xfqs * normal.x + xfqc * normal.y;
      return true;
    }
    return false;
  }

  public function computeCentroidToOut(vs : Vector<Vec2>, count : Int, out : Vec2) : Void {

    out.set(0.0, 0.0);
    var area : Float = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    var pRef : Vec2 = pool1;
    pRef.setZero();

    var e1 : Vec2 = pool2;
    var e2 : Vec2 = pool3;

    var inv3 : Float = 1.0 / 3.0;

    for (i in 0 ... count) {
    // for (int i = 0; i < count; ++i) {
      // Triangle vertices.
      var p1 : Vec2 = pRef;
      var p2 : Vec2 = vs[i];
      var p3 : Vec2 = i + 1 < count ? vs[i + 1] : vs[0];

      e1.setVec(p2).subLocal(p1);
      e2.setVec(p3).subLocal(p1);

      var D : Float = Vec2.crossVec(e1, e2);

      var triangleArea : Float = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      e1.setVec(p1).addLocalVec(p2).addLocalVec(p3).mulLocal(triangleArea * inv3);
      out.addLocalVec(e1);
    }

    // Centroid
    out.mulLocal(1.0 / area);
  }

  override public function computeMass(massData : MassData, density : Float) : Void {
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid.y = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.


    var center : Vec2 = pool1;
    center.setZero();
    var area : Float = 0.0;
    var I : Float = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    var s : Vec2 = pool2;
    s.setZero();
    // This code would put the reference point inside the polygon.
    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      s.addLocalVec(m_vertices[i]);
    }
    s.mulLocal(1.0 / m_count);

    var k_inv3 : Float = 1.0 / 3.0;

    var e1 : Vec2 = pool3;
    var e2 : Vec2 = pool4;

    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      // Triangle vertices.
      e1.setVec(m_vertices[i]).subLocal(s);
      e2.setVec(s).negateLocal().addLocalVec(i + 1 < m_count ? m_vertices[i + 1] : m_vertices[0]);

      var D : Float = Vec2.crossVec(e1, e2);

      var triangleArea : Float = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      center.x += triangleArea * k_inv3 * (e1.x + e2.x);
      center.y += triangleArea * k_inv3 * (e1.y + e2.y);

      var ex1 : Float = e1.x, ey1 = e1.y;
      var ex2 : Float = e2.x, ey2 = e2.y;

      var intx2 : Float = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
      var inty2 : Float = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

      I += (0.25 * k_inv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData.mass = density * area;

    // Center of mass
    center.mulLocal(1.0 / area);
    massData.center.setVec(center).addLocalVec(s);

    // Inertia tensor relative to the local origin (point s)
    massData.I = I * density;

    // Shift to center of mass then to original body origin.
    massData.I += massData.mass * (Vec2.dot(massData.center, massData.center));
  }

  /**
   * Validate convexity. This is a very time consuming operation.
   * 
   * @return
   */
  public function validate() : Bool {
    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var i1 : Int = i;
      var i2 : Int = i < m_count - 1 ? i1 + 1 : 0;
      var p : Vec2 = m_vertices[i1];
      var e : Vec2 = pool1.setVec(m_vertices[i2]).subLocal(p);

      for (j in 0 ... m_count) {
      // for (int j = 0; j < m_count; ++j) {
        if (j == i1 || j == i2) {
          continue;
        }

        var v : Vec2 = pool2.setVec(m_vertices[j]).subLocal(p);
        var c : Float = Vec2.crossVec(e, v);
        if (c < 0.0) {
          return false;
        }
      }
    }

    return true;
  }

  /** Get the vertices in local coordinates. */
  public function getVertices() : Vector<Vec2> {
    return m_vertices;
  }

  /** Get the edge normal vectors. There is one for each vertex. */
  public function getNormals() : Vector<Vec2> {
    return m_normals;
  }

  /** Get the centroid and apply the supplied transform. */
  public function centroid(xf : Transform) : Vec2 {
    return Transform.mul(xf, m_centroid);
  }

  /** Get the centroid and apply the supplied transform. */
  public function centroidToOut(xf : Transform, out : Vec2) : Vec2 {
    Transform.mulToOutUnsafe(xf, m_centroid, out);
    return out;
  }
}

