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
package box2d.dynamics.contacts;

import box2d.collision.Manifold;
import box2d.collision.ManifoldPoint;
import box2d.collision.WorldManifold;
import box2d.collision.shapes.Shape;
import box2d.common.Mat22;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.Fixture;
import box2d.dynamics.TimeStep;
import box2d.dynamics.contacts.VelocityConstraintPoint;

import haxe.ds.Vector;

/**
 * @author Daniel
 */
 class ContactSolver {

  public static var DEBUG_SOLVER : Bool = false;
  // public static var k_errorTol : Float = 1e-3f;
  /**
   * For each solver, this is the initial number of constraints in the array, which expands as
   * needed.
   */
  public static var INITIAL_NUM_CONSTRAINTS : Int = 256;

  /**
   * Ensure a reasonable condition number. for the block solver
   */
  public static var k_maxConditionNumber : Float = 100.0;

  public var m_step : TimeStep;
  public var m_positions : Vector<Position>;
  public var m_velocities : Vector<Velocity>;
  public var m_positionConstraints : Vector<ContactPositionConstraint>;
  public var m_velocityConstraints : Vector<ContactVelocityConstraint>;
  public var m_contacts : Vector<Contact>;
  public var m_count : Int;

  public function new() {
    m_positionConstraints = new Vector<ContactPositionConstraint>(INITIAL_NUM_CONSTRAINTS);
    m_velocityConstraints = new Vector<ContactVelocityConstraint>(INITIAL_NUM_CONSTRAINTS);
    for(i in 0 ... INITIAL_NUM_CONSTRAINTS) {
      m_positionConstraints[i] = new ContactPositionConstraint();
      m_velocityConstraints[i] = new ContactVelocityConstraint();
    }
  }

  public function init(def : ContactSolverDef) : Void {
    // System.out.println("Initializing contact solver");
    m_step = def.step;
    m_count = def.count;

    if (m_positionConstraints.length < m_count) {
      // TODO: copy array
      var old : Vector<ContactPositionConstraint> = m_positionConstraints;
      m_positionConstraints = new Vector<ContactPositionConstraint>(MathUtils.max(old.length * 2, m_count));
      // TODO: array copy
      Vector.blit(old, 0, m_positionConstraints, 0, old.length);
      // System.arraycopy(old, 0, m_positionConstraints, 0, old.length);
      for(i in old.length ... m_positionConstraints.length) {
        m_positionConstraints[i] = new ContactPositionConstraint();
      }
    }

    if (m_velocityConstraints.length < m_count) {
      var old : Vector<ContactVelocityConstraint> = m_velocityConstraints;
      m_velocityConstraints = new Vector<ContactVelocityConstraint>(MathUtils.max(old.length * 2, m_count));
      // TODO: array copy
      Vector.blit(old, 0, m_velocityConstraints, 0, old.length);
      // System.arraycopy(old, 0, m_velocityConstraints, 0, old.length);
      for(i in old.length ... m_velocityConstraints.length) {
        m_velocityConstraints[i] = new ContactVelocityConstraint();
      }
    }

    m_positions = def.positions;
    m_velocities = def.velocities;
    m_contacts = def.contacts;

    var i:Int = 0;
    while (i < m_count) {
    // for (int i = 0; i < m_count; ++i) {
      // System.out.println("contacts: " + m_count);
      var contact : Contact = m_contacts[i];

      var fixtureA : Fixture = contact.m_fixtureA;
      var fixtureB : Fixture = contact.m_fixtureB;
      var shapeA : Shape = fixtureA.getShape();
      var shapeB : Shape = fixtureB.getShape();
      var radiusA : Float = shapeA.m_radius;
      var radiusB : Float = shapeB.m_radius;
      var bodyA : Body = fixtureA.getBody();
      var bodyB : Body = fixtureB.getBody();
      var manifold : Manifold = contact.getManifold();

      var pointCount : Int = manifold.pointCount;

      var vc : ContactVelocityConstraint = m_velocityConstraints[i];
      vc.friction = contact.m_friction;
      vc.restitution = contact.m_restitution;
      vc.tangentSpeed = contact.m_tangentSpeed;
      vc.indexA = bodyA.m_islandIndex;
      vc.indexB = bodyB.m_islandIndex;
      vc.invMassA = bodyA.m_invMass;
      vc.invMassB = bodyB.m_invMass;
      vc.invIA = bodyA.m_invI;
      vc.invIB = bodyB.m_invI;
      vc.contactIndex = i;
      vc.pointCount = pointCount;
      vc.K.setZero();
      vc.normalMass.setZero();

      var pc : ContactPositionConstraint = m_positionConstraints[i];
      pc.indexA = bodyA.m_islandIndex;
      pc.indexB = bodyB.m_islandIndex;
      pc.invMassA = bodyA.m_invMass;
      pc.invMassB = bodyB.m_invMass;
      pc.localCenterA.setVec(bodyA.m_sweep.localCenter);
      pc.localCenterB.setVec(bodyB.m_sweep.localCenter);
      pc.invIA = bodyA.m_invI;
      pc.invIB = bodyB.m_invI;
      pc.localNormal.setVec(manifold.localNormal);
      pc.localPoint.setVec(manifold.localPoint);
      pc.pointCount = pointCount;
      pc.radiusA = radiusA;
      pc.radiusB = radiusB;
      pc.type = manifold.type;

      // System.out.println("contact point count: " + pointCount);
      var j:Int = 0;
      while (j < pointCount) {
      // for (int j = 0; j < pointCount; j++) {
        var cp : ManifoldPoint = manifold.points[j];
        var vcp : VelocityConstraintPoint = vc.points[j];

        if (m_step.warmStarting) {
          // assert(cp.normalImpulse == 0);
          // System.out.println("contact normal impulse: " + cp.normalImpulse);
          vcp.normalImpulse = m_step.dtRatio * cp.normalImpulse;
          vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse;
        } else {
          vcp.normalImpulse = 0;
          vcp.tangentImpulse = 0;
        }

        vcp.rA.setZero();
        vcp.rB.setZero();
        vcp.normalMass = 0;
        vcp.tangentMass = 0;
        vcp.velocityBias = 0;
        pc.localPoints[j].x = cp.localPoint.x;
        pc.localPoints[j].y = cp.localPoint.y;

        j++;
      }

      ++i;
    }
  }

  public function warmStart() : Void {
    // Warm start.
    var i : Int = 0;
    while (i < m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var vc : ContactVelocityConstraint = m_velocityConstraints[i];

      var indexA : Int = vc.indexA;
      var indexB : Int = vc.indexB;
      var mA : Float = vc.invMassA;
      var iA : Float = vc.invIA;
      var mB : Float = vc.invMassB;
      var iB : Float = vc.invIB;
      var pointCount : Int = vc.pointCount;

      var vA : Vec2 = m_velocities[indexA].v;
      var wA : Float = m_velocities[indexA].w;
      var vB : Vec2 = m_velocities[indexB].v;
      var wB : Float = m_velocities[indexB].w;

      var normal : Vec2 = vc.normal;
      var tangentx : Float = 1.0 * normal.y;
      var tangenty : Float = -1.0 * normal.x;

      var j:Int = 0;
      while (j < pointCount) {
      // for (int j = 0; j < pointCount; ++j) {
        var vcp : VelocityConstraintPoint = vc.points[j];
        var Px : Float = tangentx * vcp.tangentImpulse + normal.x * vcp.normalImpulse;
        var Py : Float = tangenty * vcp.tangentImpulse + normal.y * vcp.normalImpulse;

        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
        vB.x += Px * mB;
        vB.y += Py * mB;

        ++j;
      }
      m_velocities[indexA].w = wA;
      m_velocities[indexB].w = wB;

      ++i;
    }
  }

  // djm pooling, and from above
  private var xfA : Transform = new Transform();
  private var xfB : Transform = new Transform();
  private var worldManifold : WorldManifold = new WorldManifold();

  public function initializeVelocityConstraints() : Void {

    // Warm start.
    var i:Int = 0;
    while (i < m_count) {
      var vc : ContactVelocityConstraint = m_velocityConstraints[i];
      var pc : ContactPositionConstraint = m_positionConstraints[i];

      var radiusA : Float = pc.radiusA;
      var radiusB : Float = pc.radiusB;
      var manifold : Manifold = m_contacts[vc.contactIndex].getManifold();

      var indexA : Int = vc.indexA;
      var indexB : Int = vc.indexB;

      var mA : Float = vc.invMassA;
      var mB : Float = vc.invMassB;
      var iA : Float = vc.invIA;
      var iB : Float = vc.invIB;
      var localCenterA : Vec2 = pc.localCenterA;
      var localCenterB : Vec2 = pc.localCenterB;

      var cA : Vec2 = m_positions[indexA].c;
      var aA : Float = m_positions[indexA].a;
      var vA : Vec2 = m_velocities[indexA].v;
      var wA : Float = m_velocities[indexA].w;

      var cB : Vec2 = m_positions[indexB].c;
      var aB : Float = m_positions[indexB].a;
      var vB : Vec2 = m_velocities[indexB].v;
      var wB : Float = m_velocities[indexB].w;


      var xfAq : Rot = xfA.q;
      var xfBq : Rot = xfB.q;
      xfAq.set(aA);
      xfBq.set(aB);
      xfA.p.x = cA.x - (xfAq.c * localCenterA.x - xfAq.s * localCenterA.y);
      xfA.p.y = cA.y - (xfAq.s * localCenterA.x + xfAq.c * localCenterA.y);
      xfB.p.x = cB.x - (xfBq.c * localCenterB.x - xfBq.s * localCenterB.y);
      xfB.p.y = cB.y - (xfBq.s * localCenterB.x + xfBq.c * localCenterB.y);

      worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB);

      var vcnormal : Vec2 = vc.normal;
      vcnormal.x = worldManifold.normal.x;
      vcnormal.y = worldManifold.normal.y;

      var pointCount : Int = vc.pointCount;
      var j:Int = 0;
      while (j < pointCount) {
      // for (int j = 0; j < pointCount; ++j) {
        var vcp : VelocityConstraintPoint = vc.points[j];
        var wmPj : Vec2 = worldManifold.points[j];
        var vcprA : Vec2 = vcp.rA;
        var vcprB : Vec2 = vcp.rB;
        vcprA.x = wmPj.x - cA.x;
        vcprA.y = wmPj.y - cA.y;
        vcprB.x = wmPj.x - cB.x;
        vcprB.y = wmPj.y - cB.y;

        var rnA : Float = vcprA.x * vcnormal.y - vcprA.y * vcnormal.x;
        var rnB : Float = vcprB.x * vcnormal.y - vcprB.y * vcnormal.x;

        var kNormal : Float = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        var tangentx : Float = 1.0 * vcnormal.y;
        var tangenty : Float = -1.0 * vcnormal.x;

        var rtA : Float = vcprA.x * tangenty - vcprA.y * tangentx;
        var rtB : Float = vcprB.x * tangenty - vcprB.y * tangentx;

        var kTangent : Float = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

        vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

        // Setup a velocity bias for restitution.
        vcp.velocityBias = 0.0;
        var tempx : Float = vB.x + -wB * vcprB.y - vA.x - (-wA * vcprA.y);
        var tempy : Float = vB.y + wB * vcprB.x - vA.y - (wA * vcprA.x);
        var vRel : Float = vcnormal.x * tempx + vcnormal.y * tempy;
        if (vRel < -Settings.velocityThreshold) {
          vcp.velocityBias = -vc.restitution * vRel;
        }

        ++j;
      }

      // If we have two points, then prepare the block solver.
      if (vc.pointCount == 2) {
        var vcp1 : VelocityConstraintPoint = vc.points[0];
        var vcp2 : VelocityConstraintPoint = vc.points[1];
        var rn1A : Float = vcp1.rA.x * vcnormal.y - vcp1.rA.y * vcnormal.x;
        var rn1B : Float = vcp1.rB.x * vcnormal.y - vcp1.rB.y * vcnormal.x;
        var rn2A : Float = vcp2.rA.x * vcnormal.y - vcp2.rA.y * vcnormal.x;
        var rn2B : Float = vcp2.rB.x * vcnormal.y - vcp2.rB.y * vcnormal.x;

        var k11 : Float = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
        var k22 : Float = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
        var k12 : Float = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          vc.K.ex.x = k11;
          vc.K.ex.y = k12;
          vc.K.ey.x = k12;
          vc.K.ey.y = k22;
          vc.K.invertToOut(vc.normalMass);
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          vc.pointCount = 1;
        }
      }

      ++i;
    }
  }


  public function solveVelocityConstraints() : Void {
    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var vc : ContactVelocityConstraint = m_velocityConstraints[i];

      var indexA : Int = vc.indexA;
      var indexB : Int = vc.indexB;

      var mA : Float = vc.invMassA;
      var mB : Float = vc.invMassB;
      var iA : Float = vc.invIA;
      var iB : Float = vc.invIB;
      var pointCount : Int = vc.pointCount;

      var vA : Vec2 = m_velocities[indexA].v;
      var wA : Float = m_velocities[indexA].w;
      var vB : Vec2 = m_velocities[indexB].v;
      var wB : Float = m_velocities[indexB].w;

      var normal : Vec2 = vc.normal;
      var normalx : Float = normal.x;
      var normaly : Float = normal.y;
      var tangentx : Float = 1.0 * vc.normal.y;
      var tangenty : Float = -1.0 * vc.normal.x;
      var friction : Float = vc.friction;


      // Solve tangent constraints
      for (j in 0 ... pointCount) {
      // for (int j = 0; j < pointCount; ++j) {
        var vcp : VelocityConstraintPoint = vc.points[j];
        var a : Vec2 = vcp.rA;
        var dvx : Float = -wB * vcp.rB.y + vB.x - vA.x + wA * a.y;
        var dvy : Float = wB * vcp.rB.x + vB.y - vA.y - wA * a.x;

        // Compute tangent force
        var vt : Float = dvx * tangentx + dvy * tangenty - vc.tangentSpeed;
        var lambda : Float = vcp.tangentMass * (-vt);

        // Clamp the accumulated force
        var maxFriction : Float = friction * vcp.normalImpulse;
        var newImpulse : Float =
            MathUtils.clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - vcp.tangentImpulse;
        vcp.tangentImpulse = newImpulse;

        // Apply contact impulse
        // Vec2 P = lambda * tangent;

        var Px : Float = tangentx * lambda;
        var Py : Float = tangenty * lambda;

        // vA -= invMassA * P;
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
      }

      // Solve normal constraints
      if (vc.pointCount == 1) {
        var vcp : VelocityConstraintPoint = vc.points[0];

        // Relative velocity at contact
        // Vec2 dv = vB + Cross(wB, vcp.rB) - vA - Cross(wA, vcp.rA);

        var dvx : Float = -wB * vcp.rB.y + vB.x - vA.x + wA * vcp.rA.y;
        var dvy : Float = wB * vcp.rB.x + vB.y - vA.y - wA * vcp.rA.x;

        // Compute normal impulse
        var vn : Float = dvx * normalx + dvy * normaly;
        var lambda : Float = -vcp.normalMass * (vn - vcp.velocityBias);

        // Clamp the accumulated impulse
        var a : Float = vcp.normalImpulse + lambda;
        var newImpulse : Float = (a > 0.0 ? a : 0.0);
        lambda = newImpulse - vcp.normalImpulse;
        vcp.normalImpulse = newImpulse;

        // Apply contact impulse
        var Px : Float = normalx * lambda;
        var Py : Float = normaly * lambda;

        // vA -= invMassA * P;
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
      } else {
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on
        // Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn_0 - velocityBias
        //
        // The system is solved using the "Total enumeration method" (s. Murty). The complementary
        // constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D
        // contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be
        // tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of
        // the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the
        // impulse variable (x_i).
        //
        // Substitute:
        //
        // x = a + d
        //
        // a := old total impulse
        // x := new total impulse
        // d := incremental impulse
        //
        // For the current iteration we extend the formula for the incremental impulse
        // to compute the new total impulse:
        //
        // vn = A * d + b
        // = A * (x - a) + b
        // = A * x + b - A * a
        // = A * x + b'
        // b' = b - A * a;

        var cp1 : VelocityConstraintPoint = vc.points[0];
        var cp2 : VelocityConstraintPoint = vc.points[1];
        var cp1rA : Vec2 = cp1.rA;
        var cp1rB : Vec2 = cp1.rB;
        var cp2rA : Vec2 = cp2.rA;
        var cp2rB : Vec2 = cp2.rB;
        var ax : Float = cp1.normalImpulse;
        var ay : Float = cp2.normalImpulse;

        // Relative velocity at contact
        // Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
        var dv1x : Float = -wB * cp1rB.y + vB.x - vA.x + wA * cp1rA.y;
        var dv1y : Float = wB * cp1rB.x + vB.y - vA.y - wA * cp1rA.x;

        // Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
        var dv2x : Float = -wB * cp2rB.y + vB.x - vA.x + wA * cp2rA.y;
        var dv2y : Float = wB * cp2rB.x + vB.y - vA.y - wA * cp2rA.x;

        // Compute normal velocity
        var vn1 : Float = dv1x * normalx + dv1y * normaly;
        var vn2 : Float = dv2x * normalx + dv2y * normaly;

        var bx : Float = vn1 - cp1.velocityBias;
        var by : Float = vn2 - cp2.velocityBias;

        // Compute b'
        var R : Mat22 = vc.K;
        bx -= R.ex.x * ax + R.ey.x * ay;
        by -= R.ex.y * ax + R.ey.y * ay;

        // final float k_errorTol = 1e-3f;
        // B2_NOT_USED(k_errorTol);
        while (true) {
          //
          // Case 1: vn = 0
          //
          // 0 = A * x' + b'
          //
          // Solve for x':
          //
          // x' = - inv(A) * b'
          //
          // Vec2 x = - Mul(c.normalMass, b);
          var R1 : Mat22 = vc.normalMass;
          var xx : Float = R1.ex.x * bx + R1.ey.x * by;
          var xy : Float = R1.ex.y * bx + R1.ey.y * by;
          xx *= -1;
          xy *= -1;

          if (xx >= 0.0 && xy >= 0.0) {
            // Get the incremental impulse
            // Vec2 d = x - a;
            var dx : Float = xx - ax;
            var dy : Float = xy - ay;

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            // Vec2 P2 = d.y * normal;
            var P1x : Float = dx * normalx;
            var P1y : Float = dx * normaly;
            var P2x : Float = dy * normalx;
            var P2y : Float = dy * normaly;

            /*
             * vA -= invMassA * (P1 + P2); wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             * 
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA); dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
             * 
             * // Compute normal velocity vn1 = Dot(dv1, normal); vn2 = Dot(dv2, normal);
             * 
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); assert(Abs(vn2 - cp2.velocityBias)
             * < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              var dv1 : Vec2 = vB.add(Vec2.crossFV(wB, cp1rB).subLocal(vA).subLocal(Vec2.crossFV(wA, cp1rA)));
              var dv2 : Vec2 = vB.add(Vec2.crossFV(wB, cp2rB).subLocal(vA).subLocal(Vec2.crossFV(wA, cp2rA)));
              // Compute normal velocity
              vn1 = Vec2.dot(dv1, normal);
              vn2 = Vec2.dot(dv2, normal);

            }
            break;
          }

          //
          // Case 2: vn1 = 0 and x2 = 0
          //
          // 0 = a11 * x1' + a12 * 0 + b1'
          // vn2 = a21 * x1' + a22 * 0 + '
          //
          xx = -cp1.normalMass * bx;
          xy = 0.0;
          vn1 = 0.0;
          vn2 = vc.K.ex.y * xx + by;

          if (xx >= 0.0 && vn2 >= 0.0) {
            // Get the incremental impulse
            var dx : Float = xx - ax;
            var dy : Float = xy - ay;

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            // Vec2 P2 = d.y * normal;
            var P1x : Float = normalx * dx;
            var P1y : Float = normaly * dx;
            var P2x : Float = normalx * dy;
            var P2y : Float = normaly * dy;

            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             * 
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA);
             * 
             * // Compute normal velocity vn1 = Dot(dv1, normal);
             * 
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              var dv1 : Vec2 = vB.add(Vec2.crossFV(wB, cp1rB).subLocal(vA).subLocal(Vec2.crossFV(wA, cp1rA)));
              // Compute normal velocity
              vn1 = Vec2.dot(dv1, normal);

            }
            break;
          }

          //
          // Case 3: wB = 0 and x1 = 0
          //
          // vn1 = a11 * 0 + a12 * x2' + b1'
          // 0 = a21 * 0 + a22 * x2' + '
          //
          xx = 0.0;
          xy = -cp2.normalMass * by;
          vn1 = vc.K.ey.x * xy + bx;
          vn2 = 0.0;

          if (xy >= 0.0 && vn1 >= 0.0) {
            // Resubstitute for the incremental impulse
            var dx : Float = xx - ax;
            var dy : Float = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             * 
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            var P1x : Float = normalx * dx;
            var P1y : Float = normaly * dx;
            var P2x : Float = normalx * dy;
            var P2y : Float = normaly * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv2 = vB + Cross(wB, cp2.rB) - vA -
             * Cross(wA, cp2.rA);
             * 
             * // Compute normal velocity vn2 = Dot(dv2, normal);
             * 
             * assert(Abs(vn2 - cp2.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              var dv2 : Vec2 = vB.add(Vec2.crossFV(wB, cp2rB).subLocal(vA).subLocal(Vec2.crossFV(wA, cp2rA)));
              // Compute normal velocity
              vn2 = Vec2.dot(dv2, normal);

            }
            break;
          }

          //
          // Case 4: x1 = 0 and x2 = 0
          //
          // vn1 = b1
          // vn2 = ;
          xx = 0.0;
          xy = 0.0;
          vn1 = bx;
          vn2 = by;

          if (vn1 >= 0.0 && vn2 >= 0.0) {
            // Resubstitute for the incremental impulse
            var dx : Float = xx - ax;
            var dy : Float = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             * 
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            var P1x : Float = normalx * dx;
            var P1y : Float = normaly * dx;
            var P2x : Float = normalx * dy;
            var P2y : Float = normaly * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            break;
          }

          // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
          break;
        }
      }

      // m_velocities[indexA].v.set(vA);
      m_velocities[indexA].w = wA;
      // m_velocities[indexB].v.set(vB);
      m_velocities[indexB].w = wB;
    }
  }

  public function storeImpulses() : Void {
    for(i in 0 ... m_count) {
      var vc : ContactVelocityConstraint = m_velocityConstraints[i];
      var manifold : Manifold = m_contacts[vc.contactIndex].getManifold();

      for (j in 0 ... vc.pointCount) {
      // for (int j = 0; j < vc.pointCount; j++) {
        manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
      }
    }
  }

  /*
   * #if 0 // Sequential solver. bool ContactSolver::SolvePositionConstraints(float baumgarte) {
   * float minSeparation = 0.0f;
   * 
   * for (int i = 0; i < m_constraintCount; ++i) { ContactConstraint* c = m_constraints + i; Body*
   * bodyA = c.bodyA; Body* bodyB = c.bodyB; float invMassA = bodyA.m_mass * bodyA.m_invMass; float
   * invIA = bodyA.m_mass * bodyA.m_invI; float invMassB = bodyB.m_mass * bodyB.m_invMass; float
   * invIB = bodyB.m_mass * bodyB.m_invI;
   * 
   * Vec2 normal = c.normal;
   * 
   * // Solve normal constraints for (int j = 0; j < c.pointCount; ++j) { ContactConstraintPoint*
   * ccp = c.points + j;
   * 
   * Vec2 r1 = Mul(bodyA.GetXForm().R, ccp.localAnchorA - bodyA.GetLocalCenter()); Vec2 r2 =
   * Mul(bodyB.GetXForm().R, ccp.localAnchorB - bodyB.GetLocalCenter());
   * 
   * Vec2 p1 = bodyA.m_sweep.c + r1; Vec2 p2 = bodyB.m_sweep.c + r2; Vec2 dp = p2 - p1;
   * 
   * // Approximate the current separation. float separation = Dot(dp, normal) + ccp.separation;
   * 
   * // Track max constraint error. minSeparation = Min(minSeparation, separation);
   * 
   * // Prevent large corrections and allow slop. float C = Clamp(baumgarte * (separation +
   * _linearSlop), -_maxLinearCorrection, 0.0f);
   * 
   * // Compute normal impulse float impulse = -ccp.equalizedMass * C;
   * 
   * Vec2 P = impulse * normal;
   * 
   * bodyA.m_sweep.c -= invMassA * P; bodyA.m_sweep.a -= invIA * Cross(r1, P);
   * bodyA.SynchronizeTransform();
   * 
   * bodyB.m_sweep.c += invMassB * P; bodyB.m_sweep.a += invIB * Cross(r2, P);
   * bodyB.SynchronizeTransform(); } }
   * 
   * // We can't expect minSpeparation >= -_linearSlop because we don't // push the separation above
   * -_linearSlop. return minSeparation >= -1.5f * _linearSlop; }
   */

  private var psolver : PositionSolverManifold = new PositionSolverManifold();

  /**
   * Sequential solver.
   */
  public function solvePositionConstraints() : Bool {
    var minSeparation : Float = 0.0;

    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var pc : ContactPositionConstraint = m_positionConstraints[i];

      var indexA : Int = pc.indexA;
      var indexB : Int = pc.indexB;

      var mA : Float = pc.invMassA;
      var iA : Float = pc.invIA;
      var localCenterA : Vec2 = pc.localCenterA;
      var localCenterAx : Float = localCenterA.x;
      var localCenterAy : Float = localCenterA.y;
      var mB : Float = pc.invMassB;
      var iB : Float = pc.invIB;
      var localCenterB : Vec2 = pc.localCenterB;
      var localCenterBx : Float = localCenterB.x;
      var localCenterBy : Float = localCenterB.y;
      var pointCount : Int = pc.pointCount;

      var cA : Vec2 = m_positions[indexA].c;
      var aA : Float = m_positions[indexA].a;
      var cB : Vec2 = m_positions[indexB].c;
      var aB : Float = m_positions[indexB].a;

      // Solve normal constraints
      for (j in 0 ... pointCount) {
      // for (int j = 0; j < pointCount; ++j) {
        var xfAq : Rot = xfA.q;
        var xfBq : Rot = xfB.q;
        xfAq.set(aA);
        xfBq.set(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        var psm : PositionSolverManifold = psolver;
        psm.initialize(pc, xfA, xfB, j);
        var normal : Vec2 = psm.normal;
        var point : Vec2 = psm.point;
        var separation : Float = psm.separation;

        var rAx : Float = point.x - cA.x;
        var rAy : Float = point.y - cA.y;
        var rBx : Float = point.x - cB.x;
        var rBy : Float = point.y - cB.y;

        // Track max constraint error.
        minSeparation = MathUtils.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        var C : Float =
            MathUtils.clamp(Settings.baumgarte * (separation + Settings.linearSlop),
                -Settings.maxLinearCorrection, 0.0);

        // Compute the effective mass.
        var rnA : Float = rAx * normal.y - rAy * normal.x;
        var rnB : Float = rBx * normal.y - rBy * normal.x;
        var K : Float = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        var impulse : Float = K > 0.0 ? -C / K : 0.0;

        var Px : Float = normal.x * impulse;
        var Py : Float = normal.y * impulse;
        
        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      // m_positions[indexA].c.set(cA);
      m_positions[indexA].a = aA;

      // m_positions[indexB].c.set(cB);
      m_positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation >= -3.0 * Settings.linearSlop;
  }

  // Sequential position solver for position constraints.
  public function solveTOIPositionConstraints(toiIndexA : Int, toiIndexB : Int) : Bool {
    var minSeparation : Float = 0.0;

    for (i in 0 ... m_count) {
    // for (int i = 0; i < m_count; ++i) {
      var pc : ContactPositionConstraint = m_positionConstraints[i];

      var indexA : Int = pc.indexA;
      var indexB : Int = pc.indexB;
      var localCenterA : Vec2 = pc.localCenterA;
      var localCenterB : Vec2 = pc.localCenterB;
      var localCenterAx : Float = localCenterA.x;
      var localCenterAy : Float = localCenterA.y;
      var localCenterBx : Float = localCenterB.x;
      var localCenterBy : Float = localCenterB.y;
      var pointCount : Int = pc.pointCount;

      var mA : Float = 0.0;
      var iA : Float = 0.0;
      if (indexA == toiIndexA || indexA == toiIndexB) {
        mA = pc.invMassA;
        iA = pc.invIA;
      }

      var mB : Float = 0;
      var iB : Float = 0;
      if (indexB == toiIndexA || indexB == toiIndexB) {
        mB = pc.invMassB;
        iB = pc.invIB;
      }

      var cA : Vec2 = m_positions[indexA].c;
      var aA : Float = m_positions[indexA].a;

      var cB : Vec2 = m_positions[indexB].c;
      var aB : Float = m_positions[indexB].a;

      // Solve normal constraints
      for (j in 0 ... pointCount) {
      // for (int j = 0; j < pointCount; ++j) {
        var xfAq : Rot = xfA.q;
        var xfBq : Rot = xfB.q;
        xfAq.set(aA);
        xfBq.set(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        var psm : PositionSolverManifold = psolver;
        psm.initialize(pc, xfA, xfB, j);
        var normal : Vec2 = psm.normal;

        var point : Vec2 = psm.point;
        var separation : Float = psm.separation;

        var rAx : Float = point.x - cA.x;
        var rAy : Float = point.y - cA.y;
        var rBx : Float = point.x - cB.x;
        var rBy : Float = point.y - cB.y;

        // Track max constraint error.
        minSeparation = MathUtils.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        var C : Float =
            MathUtils.clamp(Settings.toiBaugarte * (separation + Settings.linearSlop),
                -Settings.maxLinearCorrection, 0.0);

        // Compute the effective mass.
        var rnA : Float = rAx * normal.y - rAy * normal.x;
        var rnB : Float = rBx * normal.y - rBy * normal.x;
        var K : Float = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        var impulse : Float = K > 0.0 ? -C / K : 0.0;

        var Px : Float = normal.x * impulse;
        var Py : Float = normal.y * impulse;
        
        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      // m_positions[indexA].c.set(cA);
      m_positions[indexA].a = aA;

      // m_positions[indexB].c.set(cB);
      m_positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -_linearSlop because we don't
    // push the separation above -_linearSlop.
    return minSeparation >= -1.5 * Settings.linearSlop;
  }
  
}
