package box2d.collision;

import box2d.common.Vec2;
import box2d.common.Sweep;
import box2d.common.Transform;
import box2d.common.Rot;

import haxe.ds.Vector;

enum SeparationType {
  POINTS; 
  FACE_A; 
  FACE_B;
}


class SeparationFunction {

  public var m_proxyA : DistanceProxy;
  public var m_proxyB : DistanceProxy;
  public var m_type : SeparationType;
  public var m_localPoint : Vec2 = new Vec2();
  public var m_axis : Vec2 = new Vec2();
  public var m_sweepA : Sweep;
  public var m_sweepB : Sweep;

  // djm pooling
  private var localPointA : Vec2 = new Vec2();
  private var localPointB : Vec2 = new Vec2();
  private var pointA : Vec2 = new Vec2();
  private var pointB : Vec2 = new Vec2();
  private var localPointA1 : Vec2 = new Vec2();
  private var localPointA2 : Vec2 = new Vec2();
  private var normal : Vec2 = new Vec2();
  private var localPointB1 : Vec2 = new Vec2();
  private var localPointB2 : Vec2 = new Vec2();
  private var temp : Vec2 = new Vec2();
  private var xfa : Transform = new Transform();
  private var xfb : Transform = new Transform();

  public function new () {}

  // TODO_ERIN might not need to return the separation

  public function initialize(cache : SimplexCache, proxyA : DistanceProxy, sweepA : Sweep, proxyB : DistanceProxy, sweepB : Sweep, t1 : Float) : Float {
    m_proxyA = proxyA;
    m_proxyB = proxyB;
    var count : Int = cache.count;

    m_sweepA = sweepA;
    m_sweepB = sweepB;

    m_sweepA.getTransform(xfa, t1);
    m_sweepB.getTransform(xfb, t1);

    // log.debug("initializing separation.\n" +
    // "cache: "+cache.count+"-"+cache.metric+"-"+cache.indexA+"-"+cache.indexB+"\n"
    // "distance: "+proxyA.

    if (count == 1) {
      m_type = SeparationType.POINTS;
      /*
       * Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]); Vec2 localPointB =
       * m_proxyB.GetVertex(cache.indexB[0]); Vec2 pointA = Mul(transformA, localPointA); Vec2
       * pointB = Mul(transformB, localPointB); m_axis = pointB - pointA; m_axis.Normalize();
       */
      localPointA.setVec(m_proxyA.getVertex(cache.indexA[0]));
      localPointB.setVec(m_proxyB.getVertex(cache.indexB[0]));
      Transform.mulToOutUnsafe(xfa, localPointA, pointA);
      Transform.mulToOutUnsafe(xfb, localPointB, pointB);
      m_axis.setVec(pointB).subLocal(pointA);
      var s : Float = m_axis.normalize();
      return s;
    } else if (cache.indexA[0] == cache.indexA[1]) {
      // Two points on B and one on A.
      m_type = SeparationType.FACE_B;

      localPointB1.setVec(m_proxyB.getVertex(cache.indexB[0]));
      localPointB2.setVec(m_proxyB.getVertex(cache.indexB[1]));

      temp.setVec(localPointB2).subLocal(localPointB1);
      Vec2.crossToOutUnsafe(temp, 1, m_axis);
      m_axis.normalize();

      Rot.mulToOutUnsafe(xfb.q, m_axis, normal);

      m_localPoint.setVec(localPointB1).addLocalVec(localPointB2).mulLocal(.5);
      Transform.mulToOutUnsafe(xfb, m_localPoint, pointB);

      localPointA.setVec(proxyA.getVertex(cache.indexA[0]));
      Transform.mulToOutUnsafe(xfa, localPointA, pointA);

      temp.setVec(pointA).subLocal(pointB);
      var s : Float = Vec2.dot(temp, normal);
      if (s < 0.0) {
        m_axis.negateLocal();
        s = -s;
      }
      return s;
    } else {
      // Two points on A and one or two points on B.
      m_type = SeparationType.FACE_A;

      localPointA1.setVec(m_proxyA.getVertex(cache.indexA[0]));
      localPointA2.setVec(m_proxyA.getVertex(cache.indexA[1]));

      temp.setVec(localPointA2).subLocal(localPointA1);
      Vec2.crossToOutUnsafe(temp, 1.0, m_axis);
      m_axis.normalize();

      Rot.mulToOutUnsafe(xfa.q, m_axis, normal);

      m_localPoint.setVec(localPointA1).addLocalVec(localPointA2).mulLocal(.5);
      Transform.mulToOutUnsafe(xfa, m_localPoint, pointA);

      localPointB.setVec(m_proxyB.getVertex(cache.indexB[0]));
      Transform.mulToOutUnsafe(xfb, localPointB, pointB);

      temp.setVec(pointB).subLocal(pointA);
      var s : Float = Vec2.dot(temp, normal);
      if (s < 0.0) {
        m_axis.negateLocal();
        s = -s;
      }
      return s;
    }
  }

  private var axisA : Vec2 = new Vec2();
  private var axisB : Vec2 = new Vec2();

  // float FindMinSeparation(int* indexA, int* indexB, float t) const
  public function findMinSeparation(indexes : Vector<Int>, t : Float) : Float {

    m_sweepA.getTransform(xfa, t);
    m_sweepB.getTransform(xfb, t);

    switch (m_type) {
      case POINTS: {
        Rot.mulTransUnsafe2(xfa.q, m_axis, axisA);
        Rot.mulTransUnsafe2(xfb.q, m_axis.negateLocal(), axisB);
        m_axis.negateLocal();

        indexes[0] = m_proxyA.getSupport(axisA);
        indexes[1] = m_proxyB.getSupport(axisB);

        localPointA.setVec(m_proxyA.getVertex(indexes[0]));
        localPointB.setVec(m_proxyB.getVertex(indexes[1]));

        Transform.mulToOutUnsafe(xfa, localPointA, pointA);
        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

        var separation : Float = Vec2.dot(pointB.subLocal(pointA), m_axis);
        return separation;
      }
      case FACE_A: {
        Rot.mulToOutUnsafe(xfa.q, m_axis, normal);
        Transform.mulToOutUnsafe(xfa, m_localPoint, pointA);

        Rot.mulTransUnsafe2(xfb.q, normal.negateLocal(), axisB);
        normal.negateLocal();

        indexes[0] = -1;
        indexes[1] = m_proxyB.getSupport(axisB);

        localPointB.setVec(m_proxyB.getVertex(indexes[1]));
        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

        var separation : Float = Vec2.dot(pointB.subLocal(pointA), normal);
        return separation;
      }
      case FACE_B: {
        Rot.mulToOutUnsafe(xfb.q, m_axis, normal);
        Transform.mulToOutUnsafe(xfb, m_localPoint, pointB);

        Rot.mulTransUnsafe2(xfa.q, normal.negateLocal(), axisA);
        normal.negateLocal();

        indexes[1] = -1;
        indexes[0] = m_proxyA.getSupport(axisA);

        localPointA.setVec(m_proxyA.getVertex(indexes[0]));
        Transform.mulToOutUnsafe(xfa, localPointA, pointA);

        var separation : Float = Vec2.dot(pointA.subLocal(pointB), normal);
        return separation;
      };
      default:
        indexes[0] = -1;
        indexes[1] = -1;
        return 0;
    }
  }

  public function evaluate(indexA : Int, indexB : Int, t : Float) : Float {
    m_sweepA.getTransform(xfa, t);
    m_sweepB.getTransform(xfb, t);

    switch (m_type) {
      case POINTS: {
        localPointA.setVec(m_proxyA.getVertex(indexA));
        localPointB.setVec(m_proxyB.getVertex(indexB));

        Transform.mulToOutUnsafe(xfa, localPointA, pointA);
        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

        var separation : Float = Vec2.dot(pointB.subLocal(pointA), m_axis);
        return separation;
      }
      case FACE_A: {
        Rot.mulToOutUnsafe(xfa.q, m_axis, normal);
        Transform.mulToOutUnsafe(xfa, m_localPoint, pointA);

        localPointB.setVec(m_proxyB.getVertex(indexB));
        Transform.mulToOutUnsafe(xfb, localPointB, pointB);
        var separation : Float = Vec2.dot(pointB.subLocal(pointA), normal);
        return separation;
      }
      case FACE_B: {
        Rot.mulToOutUnsafe(xfb.q, m_axis, normal);
        Transform.mulToOutUnsafe(xfb, m_localPoint, pointB);

        localPointA.setVec(m_proxyA.getVertex(indexA));
        Transform.mulToOutUnsafe(xfa, localPointA, pointA);

        var separation : Float = Vec2.dot(pointA.subLocal(pointB), normal);
        return separation;
      }
      default:
        return 0;
    }
  }
}

