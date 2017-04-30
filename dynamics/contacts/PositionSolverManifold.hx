package box2d.dynamics.contacts;

import box2d.common.Vec2;
import box2d.common.Rot;
import box2d.common.Transform;

class PositionSolverManifold {

  public var normal : Vec2 = new Vec2();
  public var point : Vec2 = new Vec2();
  public var separation : Float;

  public function new() {}

  public function initialize(pc : ContactPositionConstraint, xfA : Transform, xfB : Transform, index : Int) : Void {

    var xfAq : Rot = xfA.q;
    var xfBq : Rot = xfB.q;
    var pcLocalPointsI : Vec2 = pc.localPoints[index];
    switch (pc.type) {
      case CIRCLES: {
        // Transform.mulToOutUnsafe(xfA, pc.localPoint, pointA);
        // Transform.mulToOutUnsafe(xfB, pc.localPoints[0], pointB);
        // normal.set(pointB).subLocal(pointA);
        // normal.normalize();
        //
        // point.set(pointA).addLocal(pointB).mulLocal(.5f);
        // temp.set(pointB).subLocal(pointA);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        var plocalPoint : Vec2 = pc.localPoint;
        var pLocalPoints0 : Vec2 = pc.localPoints[0];
        var pointAx : Float = (xfAq.c * plocalPoint.x - xfAq.s * plocalPoint.y) + xfA.p.x;
        var pointAy : Float = (xfAq.s * plocalPoint.x + xfAq.c * plocalPoint.y) + xfA.p.y;
        var pointBx : Float = (xfBq.c * pLocalPoints0.x - xfBq.s * pLocalPoints0.y) + xfB.p.x;
        var pointBy : Float = (xfBq.s * pLocalPoints0.x + xfBq.c * pLocalPoints0.y) + xfB.p.y;
        normal.x = pointBx - pointAx;
        normal.y = pointBy - pointAy;
        normal.normalize();

        point.x = (pointAx + pointBx) * .5;
        point.y = (pointAy + pointBy) * .5;
        var tempx : Float = pointBx - pointAx;
        var tempy : Float = pointBy - pointAy;
        separation = tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
      }

      case FACE_A: {
        // Rot.mulToOutUnsafe(xfAq, pc.localNormal, normal);
        // Transform.mulToOutUnsafe(xfA, pc.localPoint, planePoint);
        //
        // Transform.mulToOutUnsafe(xfB, pc.localPoints[index], clipPoint);
        // temp.set(clipPoint).subLocal(planePoint);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        // point.set(clipPoint);
        var pcLocalNormal : Vec2 = pc.localNormal;
        var pcLocalPoint : Vec2 = pc.localPoint;
        normal.x = xfAq.c * pcLocalNormal.x - xfAq.s * pcLocalNormal.y;
        normal.y = xfAq.s * pcLocalNormal.x + xfAq.c * pcLocalNormal.y;
        var planePointx : Float = (xfAq.c * pcLocalPoint.x - xfAq.s * pcLocalPoint.y) + xfA.p.x;
        var planePointy : Float = (xfAq.s * pcLocalPoint.x + xfAq.c * pcLocalPoint.y) + xfA.p.y;

        var clipPointx : Float = (xfBq.c * pcLocalPointsI.x - xfBq.s * pcLocalPointsI.y) + xfB.p.x;
        var clipPointy : Float = (xfBq.s * pcLocalPointsI.x + xfBq.c * pcLocalPointsI.y) + xfB.p.y;
        var tempx : Float = clipPointx - planePointx;
        var tempy : Float = clipPointy - planePointy;
        separation = tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
      }

      case FACE_B: {
        // Rot.mulToOutUnsafe(xfBq, pc.localNormal, normal);
        // Transform.mulToOutUnsafe(xfB, pc.localPoint, planePoint);
        //
        // Transform.mulToOutUnsafe(xfA, pcLocalPointsI, clipPoint);
        // temp.set(clipPoint).subLocal(planePoint);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        // point.set(clipPoint);
        //
        // // Ensure normal points from A to B
        // normal.negateLocal();
        var pcLocalNormal : Vec2 = pc.localNormal;
        var pcLocalPoint : Vec2 = pc.localPoint;
        normal.x = xfBq.c * pcLocalNormal.x - xfBq.s * pcLocalNormal.y;
        normal.y = xfBq.s * pcLocalNormal.x + xfBq.c * pcLocalNormal.y;
        var planePointx : Float = (xfBq.c * pcLocalPoint.x - xfBq.s * pcLocalPoint.y) + xfB.p.x;
        var planePointy : Float = (xfBq.s * pcLocalPoint.x + xfBq.c * pcLocalPoint.y) + xfB.p.y;

        var clipPointx : Float = (xfAq.c * pcLocalPointsI.x - xfAq.s * pcLocalPointsI.y) + xfA.p.x;
        var clipPointy : Float = (xfAq.s * pcLocalPointsI.x + xfAq.c * pcLocalPointsI.y) + xfA.p.y;
        var tempx : Float = clipPointx - planePointx;
        var tempy : Float = clipPointy - planePointy;
        separation = tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
        normal.x *= -1;
        normal.y *= -1;
      }
    }
  }
}