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
 * A circle shape.
 */
 class CircleShape extends Shape {

  public var m_p : Vec2;

  public function new() {
    super(ShapeType.CIRCLE);
    m_p = new Vec2();
    m_radius = 0;
  }

  override public function clone() : Shape {
    var shape : CircleShape = new CircleShape();
    shape.m_p.x = m_p.x;
    shape.m_p.y = m_p.y;
    shape.m_radius = m_radius;
    return shape;
  }

  override public function getChildCount() : Int {
    return 1;
  }

  /**
   * Get the supporting vertex index in the given direction.
   * 
   * @param d
   * @return
   */
  public function getSupport(d : Vec2) : Int {
    return 0;
  }

  /**
   * Get the supporting vertex in the given direction.
   * 
   * @param d
   * @return
   */
  public function getSupportVertex(d : Vec2) : Vec2 {
    return m_p;
  }

  /**
   * Get the vertex count.
   * 
   * @return
   */
  public function getVertexCount() : Int {
    return 1;
  }

  /**
   * Get a vertex by index.
   * 
   * @param index
   * @return
   */
  public function getVertex(index : Int) : Vec2 {
    return m_p;
  }

  override public function testPoint(transform : Transform, p : Vec2) : Bool {
    // Rot.mulToOutUnsafe(transform.q, m_p, center);
    // center.addLocal(transform.p);
    //
    // final Vec2 d = center.subLocal(p).negateLocal();
    // return Vec2.dot(d, d) <= m_radius * m_radius;
    var q : Rot = transform.q;
    var tp : Vec2 = transform.p;
    var centerx : Float = -(q.c * m_p.x - q.s * m_p.y + tp.x - p.x);
    var centery : Float = -(q.s * m_p.x + q.c * m_p.y + tp.y - p.y);

    return centerx * centerx + centery * centery <= m_radius * m_radius;
  }

  override public function computeDistanceToOut(xf : Transform, p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    var xfq : Rot = xf.q;
    var centerx : Float = xfq.c * m_p.x - xfq.s * m_p.y + xf.p.x;
    var centery : Float = xfq.s * m_p.x + xfq.c * m_p.y + xf.p.y;
    var dx : Float = p.x - centerx;
    var dy : Float = p.y - centery;
    var d1 : Float = MathUtils.sqrt(dx * dx + dy * dy);
    normalOut.x = dx * 1 / d1;
    normalOut.y = dy * 1 / d1;
    return d1 - m_radius;
  }

  // Collision Detection in Interactive 3D Environments by Gino van den Bergen
  // From Section 3.1.2
  // x = s + a * r
  // norm(x) = radius
  override public function raycast(output : RayCastOutput, input : RayCastInput, transform : Transform, childIndex : Int) : Bool {

    var inputp1 : Vec2 = input.p1;
    var inputp2 : Vec2 = input.p2;
    var tq : Rot = transform.q;
    var tp : Vec2 = transform.p;

    // Rot.mulToOutUnsafe(transform.q, m_p, position);
    // position.addLocal(transform.p);
    var positionx : Float = tq.c * m_p.x - tq.s * m_p.y + tp.x;
    var positiony : Float = tq.s * m_p.x + tq.c * m_p.y + tp.y;

    var sx : Float = inputp1.x - positionx;
    var sy : Float = inputp1.y - positiony;
    // final float b = Vec2.dot(s, s) - m_radius * m_radius;
    var b : Float = sx * sx + sy * sy - m_radius * m_radius;

    // Solve quadratic equation.
    var rx : Float = inputp2.x - inputp1.x;
    var ry : Float = inputp2.y - inputp1.y;
    // final float c = Vec2.dot(s, r);
    // final float rr = Vec2.dot(r, r);
    var c : Float = sx * rx + sy * ry;
    var rr : Float = rx * rx + ry * ry;
    var sigma : Float = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0 || rr < Settings.EPSILON) {
      return false;
    }

    // Find the point of intersection of the line with the circle.
    var a : Float = -(c + MathUtils.sqrt(sigma));

    // Is the intersection point on the segment?
    if (0.0 <= a && a <= input.maxFraction * rr) {
      a /= rr;
      output.fraction = a;
      output.normal.x = rx * a + sx;
      output.normal.y = ry * a + sy;
      output.normal.normalize();
      return true;
    }

    return false;
  }

  override public function computeAABB(aabb : AABB, transform : Transform, childIndex : Int) : Void {
    var tq : Rot = transform.q;
    var tp : Vec2 = transform.p;
    var px : Float = tq.c * m_p.x - tq.s * m_p.y + tp.x;
    var py : Float = tq.s * m_p.x + tq.c * m_p.y + tp.y;

    aabb.lowerBound.x = px - m_radius;
    aabb.lowerBound.y = py - m_radius;
    aabb.upperBound.x = px + m_radius;
    aabb.upperBound.y = py + m_radius;
  }

  override public function computeMass(massData : MassData, density : Float) : Void {
    massData.mass = density * Settings.PI * m_radius * m_radius;
    massData.center.x = m_p.x;
    massData.center.y = m_p.y;

    // inertia about the local origin
    // massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_p, m_p));
    massData.I = massData.mass * (0.5 * m_radius * m_radius + (m_p.x * m_p.x + m_p.y * m_p.y));
  }
}

