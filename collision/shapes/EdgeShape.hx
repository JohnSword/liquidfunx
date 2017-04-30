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

/**
 * A line segment (edge) shape. These can be connected in chains or loops to other edge shapes. The
 * connectivity information is used to ensure correct contact normals.
 * 
 * @author Daniel
 */
 class EdgeShape extends Shape {

  /**
   * edge vertex 1
   */
  public var m_vertex1 : Vec2 = new Vec2();
  /**
   * edge vertex 2
   */
  public var m_vertex2 : Vec2 = new Vec2();

  /**
   * optional adjacent vertex 1. Used for smooth collision
   */
  public var m_vertex0 : Vec2 = new Vec2();
  /**
   * optional adjacent vertex 2. Used for smooth collision
   */
  public var m_vertex3 : Vec2 = new Vec2();
  public var m_hasVertex3 : Bool = false;
  public var m_hasVertex0 : Bool = false;


  public function new() {
    super(ShapeType.EDGE);
    m_radius = Settings.polygonRadius;
  }

  override public function getChildCount() : Int {
    return 1;
  }

  public function set(v1 : Vec2, v2 : Vec2) : Void {
    m_vertex1.setVec(v1);
    m_vertex2.setVec(v2);
    m_hasVertex0 = m_hasVertex3 = false;
  }

  override public function testPoint(xf : Transform, p : Vec2) : Bool {
    return false;
  }

  // for pooling
  private var normal : Vec2 = new Vec2();

  override public function computeDistanceToOut(xf : Transform, p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    var xfqc : Float = xf.q.c;
    var xfqs : Float = xf.q.s;
    var xfpx : Float = xf.p.x;
    var xfpy : Float = xf.p.y;
    var v1x : Float = (xfqc * m_vertex1.x - xfqs * m_vertex1.y) + xfpx;
    var v1y : Float = (xfqs * m_vertex1.x + xfqc * m_vertex1.y) + xfpy;
    var v2x : Float = (xfqc * m_vertex2.x - xfqs * m_vertex2.y) + xfpx;
    var v2y : Float = (xfqs * m_vertex2.x + xfqc * m_vertex2.y) + xfpy;

    var dx : Float = p.x - v1x;
    var dy : Float = p.y - v1y;
    var sx : Float = v2x - v1x;
    var sy : Float = v2y - v1y;
    var ds : Float = dx * sx + dy * sy;
    if (ds > 0) {
      var s2 : Float = sx * sx + sy * sy;
      if (ds > s2) {
        dx = p.x - v2x;
        dy = p.y - v2y;
      } else {
        dx -= ds / s2 * sx;
        dy -= ds / s2 * sy;
      }
    }

    var d1 : Float = MathUtils.sqrt(dx * dx + dy * dy);
    if (d1 > 0) {
      normalOut.x = 1 / d1 * dx;
      normalOut.y = 1 / d1 * dy;
    } else {
      normalOut.x = 0;
      normalOut.y = 0;
    }
    return d1;
  }

  // p = p1 + t * d
  // v = v1 + s * e
  // p1 + t * d = v1 + s * e
  // s * e - t * d = p1 - v1
  override public function raycast(output : RayCastOutput, input : RayCastInput, xf : Transform, childIndex : Int) : Bool {

    var tempx : Float, tempy : Float;
    var v1 : Vec2 = m_vertex1;
    var v2 : Vec2 = m_vertex2;
    var xfq : Rot = xf.q;
    var xfp : Vec2 = xf.p;

    // Put the ray into the edge's frame of reference.
    // b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
    // b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
    tempx = input.p1.x - xfp.x;
    tempy = input.p1.y - xfp.y;
    var p1x : Float = xfq.c * tempx + xfq.s * tempy;
    var p1y : Float = -xfq.s * tempx + xfq.c * tempy;

    tempx = input.p2.x - xfp.x;
    tempy = input.p2.y - xfp.y;
    var p2x : Float = xfq.c * tempx + xfq.s * tempy;
    var p2y : Float = -xfq.s * tempx + xfq.c * tempy;

    var dx : Float = p2x - p1x;
    var dy : Float = p2y - p1y;

    // final Vec2 normal = pool2.set(v2).subLocal(v1);
    // normal.set(normal.y, -normal.x);
    normal.x = v2.y - v1.y;
    normal.y = v1.x - v2.x;
    normal.normalize();
    var normalx : Float = normal.x;
    var normaly : Float = normal.y;

    // q = p1 + t * d
    // dot(normal, q - v1) = 0
    // dot(normal, p1 - v1) + t * dot(normal, d) = 0
    tempx = v1.x - p1x;
    tempy = v1.y - p1y;
    var numerator : Float = normalx * tempx + normaly * tempy;
    var denominator : Float = normalx * dx + normaly * dy;

    if (denominator == 0.0) {
      return false;
    }

    var t : Float = numerator / denominator;
    if (t < 0.0 || 1.0 < t) {
      return false;
    }

    // Vec2 q = p1 + t * d;
    var qx : Float = p1x + t * dx;
    var qy : Float = p1y + t * dy;

    // q = v1 + s * r
    // s = dot(q - v1, r) / dot(r, r)
    // Vec2 r = v2 - v1;
    var rx : Float = v2.x - v1.x;
    var ry : Float = v2.y - v1.y;
    var rr : Float = rx * rx + ry * ry;
    if (rr == 0.0) {
      return false;
    }
    tempx = qx - v1.x;
    tempy = qy - v1.y;
    // float s = Vec2.dot(pool5, r) / rr;
    var s : Float = (tempx * rx + tempy * ry) / rr;
    if (s < 0.0 || 1.0 < s) {
      return false;
    }

    output.fraction = t;
    if (numerator > 0.0) {
      // output.normal = -b2Mul(xf.q, normal);
      output.normal.x = -xfq.c * normal.x + xfq.s * normal.y;
      output.normal.y = -xfq.s * normal.x - xfq.c * normal.y;
    } else {
      // output->normal = b2Mul(xf.q, normal);
      output.normal.x = xfq.c * normal.x - xfq.s * normal.y;
      output.normal.y = xfq.s * normal.x + xfq.c * normal.y;
    }
    return true;
  }

  override public function computeAABB(aabb : AABB, xf : Transform, childIndex : Int) : Void {
    var lowerBound : Vec2 = aabb.lowerBound;
    var upperBound : Vec2 = aabb.upperBound;
    var xfq : Rot = xf.q;

    var v1x : Float = (xfq.c * m_vertex1.x - xfq.s * m_vertex1.y) + xf.p.x;
    var v1y : Float = (xfq.s * m_vertex1.x + xfq.c * m_vertex1.y) + xf.p.y;
    var v2x : Float = (xfq.c * m_vertex2.x - xfq.s * m_vertex2.y) + xf.p.x;
    var v2y : Float = (xfq.s * m_vertex2.x + xfq.c * m_vertex2.y) + xf.p.y;

    lowerBound.x = v1x < v2x ? v1x : v2x;
    lowerBound.y = v1y < v2y ? v1y : v2y;
    upperBound.x = v1x > v2x ? v1x : v2x;
    upperBound.y = v1y > v2y ? v1y : v2y;

    lowerBound.x -= m_radius;
    lowerBound.y -= m_radius;
    upperBound.x += m_radius;
    upperBound.y += m_radius;
  }

  override public function computeMass(massData : MassData, density : Float) : Void {
    massData.mass = 0.0;
    massData.center.setVec(m_vertex1).addLocalVec(m_vertex2).mulLocal(0.5);
    massData.I = 0.0;
  }

  override public function clone() : Shape {
    var edge : EdgeShape = new EdgeShape();
    edge.m_radius = this.m_radius;
    edge.m_hasVertex0 = this.m_hasVertex0;
    edge.m_hasVertex3 = this.m_hasVertex3;
    edge.m_vertex0.setVec(this.m_vertex0);
    edge.m_vertex1.setVec(this.m_vertex1);
    edge.m_vertex2.setVec(this.m_vertex2);
    edge.m_vertex3.setVec(this.m_vertex3);
    return edge;
  }
}

