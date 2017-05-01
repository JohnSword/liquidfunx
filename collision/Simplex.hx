package box2d.collision;

import box2d.collision.SimplexVertex;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.common.Transform;
import box2d.common.MathUtils;

class Simplex {

    public var m_v1 : SimplexVertex = new SimplexVertex();
    public var m_v2 : SimplexVertex = new SimplexVertex();
    public var m_v3 : SimplexVertex = new SimplexVertex();
    public var vertices : Array<SimplexVertex>;
    public var m_count : Int;

    public function new () {
        vertices = new Array <SimplexVertex> ();
        vertices[0] = m_v1;
        vertices[1] = m_v2;
        vertices[2] = m_v3;
    }

    public function readCache(cache : SimplexCache, proxyA : DistanceProxy, transformA : Transform, proxyB : DistanceProxy, transformB : Transform) : Void {
      // Copy data from cache.
      m_count = cache.count;

      for(i in 0 ... m_count) {
        var v : SimplexVertex = vertices[i];
        v.indexA = cache.indexA[i];
        v.indexB = cache.indexB[i];
        var wALocal : Vec2 = proxyA.getVertex(v.indexA);
        var wBLocal : Vec2 = proxyB.getVertex(v.indexB);
        Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
        Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
        v.w.setVec(v.wB).subLocal(v.wA);
        v.a = 0.0;
      }

      // Compute the new simplex metric, if it is substantially different than
      // old metric then flush the simplex.
      if (m_count > 1) {
        var metric1 : Float = cache.metric;
        var metric2 : Float = getMetric();
        if (metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Settings.EPSILON) {
          // Reset the simplex.
          m_count = 0;
        }
      }

      // If the cache is empty or invalid ...
      if (m_count == 0) {
        var v : SimplexVertex = vertices[0];
        v.indexA = 0;
        v.indexB = 0;
        var wALocal : Vec2 = proxyA.getVertex(0);
        var wBLocal : Vec2 = proxyB.getVertex(0);
        Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
        Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
        v.w.setVec(v.wB).subLocal(v.wA);
        m_count = 1;
      }
    }

    public function writeCache(cache : SimplexCache) : Void {
      cache.metric = getMetric();
      cache.count = m_count;

      for(i in 0 ... m_count) {
        cache.indexA[i] = (vertices[i].indexA);
        cache.indexB[i] = (vertices[i].indexB);
      }
    }

    private var e12 : Vec2 = new Vec2();

    public function getSearchDirection(out : Vec2) : Void {
      switch (m_count) {
      case 1:
        out.setVec(m_v1.w).negateLocal();
        return;
      case 2:
        e12.setVec(m_v2.w).subLocal(m_v1.w);
        // use out for a temp variable real quick
        out.setVec(m_v1.w).negateLocal();
        var sgn : Float = Vec2.crossVec(e12, out);

        if (sgn > 0) {
          // Origin is left of e12.
          Vec2.crossToOutUnsafeFVV(1, e12, out);
          return;
        } else {
          // Origin is right of e12.
          Vec2.crossToOutUnsafe(e12, 1, out);
          return;
        }
      default:
        out.setZero();
        return;
      }
    }

    // djm pooled
    private var case2 : Vec2 = new Vec2();
    private var case22 : Vec2 = new Vec2();

    /**
     * this returns pooled objects. don't keep or modify them
     * 
     * @return
     */
    public function getClosestPoint(out : Vec2) : Void {
      switch (m_count) {
      case 0:
        out.setZero();
        return;
      case 1:
        out.setVec(m_v1.w);
        return;
      case 2:
        case22.setVec(m_v2.w).mulLocal(m_v2.a);
        case2.setVec(m_v1.w).mulLocal(m_v1.a).addLocalVec(case22);
        out.setVec(case2);
        return;
      case 3:
        out.setZero();
        return;
      default:
        out.setZero();
        return;
      }
    }

    // djm pooled, and from above
    private var case3 : Vec2 = new Vec2();
    private var case33 : Vec2 = new Vec2();

    public function getWitnessPoints(pA : Vec2, pB : Vec2) : Void {
      switch (m_count) {
      case 0:

      case 1:
        pA.setVec(m_v1.wA);
        pB.setVec(m_v1.wB);

      case 2:
        case2.setVec(m_v1.wA).mulLocal(m_v1.a);
        pA.setVec(m_v2.wA).mulLocal(m_v2.a).addLocalVec(case2);
        // m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
        // *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
        case2.setVec(m_v1.wB).mulLocal(m_v1.a);
        pB.setVec(m_v2.wB).mulLocal(m_v2.a).addLocalVec(case2);

      case 3:
        pA.setVec(m_v1.wA).mulLocal(m_v1.a);
        case3.setVec(m_v2.wA).mulLocal(m_v2.a);
        case33.setVec(m_v3.wA).mulLocal(m_v3.a);
        pA.addLocalVec(case3).addLocalVec(case33);
        pB.setVec(pA);
        // *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
        // *pB = *pA;

      default:
      }
    }

    // djm pooled, from above
    public function getMetric() : Float {
      switch (m_count) {
      case 0:
        return 0.0;

      case 1:
        return 0.0;

      case 2:
        return MathUtils.distance(m_v1.w, m_v2.w);

      case 3:
        case3.setVec(m_v2.w).subLocal(m_v1.w);
        case33.setVec(m_v3.w).subLocal(m_v1.w);
        // return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
        return Vec2.crossVec(case3, case33);

      default:
        return 0.0;
      }
    }

    // djm pooled from above
    /**
     * Solve a line segment using barycentric coordinates.
     */
    public function solve2() : Void {
      // Solve a line segment using barycentric coordinates.
      //
      // p = a1 * w1 + a2 * w2
      // a1 + a2 = 1
      //
      // The vector from the origin to the closest point on the line is
      // perpendicular to the line.
      // e12 = w2 - w1
      // dot(p, e) = 0
      // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
      //
      // 2-by-2 linear system
      // [1 1 ][a1] = [1]
      // [w1.e12 w2.e12][a2] = [0]
      //
      // Define
      // d12_1 = dot(w2, e12)
      // d12_2 = -dot(w1, e12)
      // d12 = d12_1 + d12_2
      //
      // Solution
      // a1 = d12_1 / d12
      // a2 = d12_2 / d12
      var w1 : Vec2 = m_v1.w;
      var w2 : Vec2 = m_v2.w;
      e12.setVec(w2).subLocal(w1);

      // w1 region
      var d12_2 : Float = -Vec2.dot(w1, e12);
      if (d12_2 <= 0.0) {
        // a2 <= 0, so we clamp it to 0
        m_v1.a = 1.0;
        m_count = 1;
        return;
      }

      // w2 region
      var d12_1 : Float = Vec2.dot(w2, e12);
      if (d12_1 <= 0.0) {
        // a1 <= 0, so we clamp it to 0
        m_v2.a = 1.0;
        m_count = 1;
        m_v1.set(m_v2);
        return;
      }

      // Must be in e12 region.
      var inv_d12 : Float = 1.0 / (d12_1 + d12_2);
      m_v1.a = d12_1 * inv_d12;
      m_v2.a = d12_2 * inv_d12;
      m_count = 2;
    }

    // djm pooled, and from above
    private var e13 : Vec2 = new Vec2();
    private var e23 : Vec2 = new Vec2();
    private var w1 : Vec2 = new Vec2();
    private var w2 : Vec2 = new Vec2();
    private var w3 : Vec2 = new Vec2();

    /**
     * Solve a line segment using barycentric coordinates.<br/>
     * Possible regions:<br/>
     * - points[2]<br/>
     * - edge points[0]-points[2]<br/>
     * - edge points[1]-points[2]<br/>
     * - inside the triangle
     */
    public function solve3() : Void {
      w1.setVec(m_v1.w);
      w2.setVec(m_v2.w);
      w3.setVec(m_v3.w);

      // Edge12
      // [1 1 ][a1] = [1]
      // [w1.e12 w2.e12][a2] = [0]
      // a3 = 0
      e12.setVec(w2).subLocal(w1);
      var w1e12 : Float = Vec2.dot(w1, e12);
      var w2e12 : Float = Vec2.dot(w2, e12);
      var d12_1 : Float = w2e12;
      var d12_2 : Float = -w1e12;

      // Edge13
      // [1 1 ][a1] = [1]
      // [w1.e13 w3.e13][a3] = [0]
      // a2 = 0
      e13.setVec(w3).subLocal(w1);
      var w1e13 : Float = Vec2.dot(w1, e13);
      var w3e13 : Float = Vec2.dot(w3, e13);
      var d13_1 : Float = w3e13;
      var d13_2 : Float = -w1e13;

      // Edge23
      // [1 1 ][a2] = [1]
      // [w2.e23 w3.e23][a3] = [0]
      // a1 = 0
      e23.setVec(w3).subLocal(w2);
      var w2e23 : Float = Vec2.dot(w2, e23);
      var w3e23 : Float = Vec2.dot(w3, e23);
      var d23_1 : Float = w3e23;
      var d23_2 : Float = -w2e23;

      // Triangle123
      var n123 : Float = Vec2.crossVec(e12, e13);

      var d123_1 : Float = n123 * Vec2.crossVec(w2, w3);
      var d123_2 : Float = n123 * Vec2.crossVec(w3, w1);
      var d123_3 : Float = n123 * Vec2.crossVec(w1, w2);

      // w1 region
      if (d12_2 <= 0.0 && d13_2 <= 0.0) {
        m_v1.a = 1.0;
        m_count = 1;
        return;
      }

      // e12
      if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
        var inv_d12 : Float = 1.0 / (d12_1 + d12_2);
        m_v1.a = d12_1 * inv_d12;
        m_v2.a = d12_2 * inv_d12;
        m_count = 2;
        return;
      }

      // e13
      if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
        var inv_d13 : Float = 1.0 / (d13_1 + d13_2);
        m_v1.a = d13_1 * inv_d13;
        m_v3.a = d13_2 * inv_d13;
        m_count = 2;
        m_v2.set(m_v3);
        return;
      }

      // w2 region
      if (d12_1 <= 0.0 && d23_2 <= 0.0) {
        m_v2.a = 1.0;
        m_count = 1;
        m_v1.set(m_v2);
        return;
      }

      // w3 region
      if (d13_1 <= 0.0 && d23_1 <= 0.0) {
        m_v3.a = 1.0;
        m_count = 1;
        m_v1.set(m_v3);
        return;
      }

      // e23
      if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
        var inv_d23 : Float = 1.0 / (d23_1 + d23_2);
        m_v2.a = d23_1 * inv_d23;
        m_v3.a = d23_2 * inv_d23;
        m_count = 2;
        m_v1.set(m_v3);
        return;
      }

      // Must be in triangle123
      var inv_d123 : Float = 1.0 / (d123_1 + d123_2 + d123_3);
      m_v1.a = d123_1 * inv_d123;
      m_v2.a = d123_2 * inv_d123;
      m_v3.a = d123_3 * inv_d123;
      m_count = 3;
    }
  }