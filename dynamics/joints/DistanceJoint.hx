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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package box2d.dynamics.joints;

import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/**
 * A distance joint constrains two points on two bodies to remain at a fixed distance from each
 * other. You can view this as a massless, rigid rod.
 */
 class DistanceJoint extends Joint {

  private var m_frequencyHz : Float;
  private var m_dampingRatio : Float;
  private var m_bias : Float;

  // Solver shared
  private var m_localAnchorA : Vec2;
  private var m_localAnchorB : Vec2;
  private var m_gamma : Float;
  private var m_impulse : Float;
  private var m_length : Float;

  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_u : Vec2 = new Vec2();
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;
  private var m_mass : Float;

  public function new(argWorld : IWorldPool, def : DistanceJointDef) {
    super(argWorld, def);
    m_localAnchorA = def.localAnchorA.clone();
    m_localAnchorB = def.localAnchorB.clone();
    m_length = def.length;
    m_impulse = 0.0;
    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;
    m_gamma = 0.0;
    m_bias = 0.0;
  }

  public function setFrequency(hz : Float) : Void {
    m_frequencyHz = hz;
  }

  public function getFrequency() : Float {
    return m_frequencyHz;
  }

  public function getLength() : Float {
    return m_length;
  }

  public function setLength(argLength : Float) : Void {
    m_length = argLength;
  }

  public function setDampingRatio(damp : Float) : Void {
    m_dampingRatio = damp;
  }

  public function getDampingRatio() : Float {
    return m_dampingRatio;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  public function getLocalAnchorA() : Vec2 {
    return m_localAnchorA;
  }

  public function getLocalAnchorB() : Vec2 {
    return m_localAnchorB;
  }

  /**
   * Get the reaction force given the inverse time step. Unit is N.
   */
  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.x = m_impulse * m_u.x * inv_dt;
    argOut.y = m_impulse * m_u.y * inv_dt;
  }

  /**
   * Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
   * distance joint.
   */
  override public function getReactionTorque(inv_dt : Float) : Float {
    return 0.0;
  }

  override public function initVelocityConstraints(data : SolverData) : Void {

    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterA.setVec(m_bodyA.m_sweep.localCenter);
    m_localCenterB.setVec(m_bodyB.m_sweep.localCenter);
    m_invMassA = m_bodyA.m_invMass;
    m_invMassB = m_bodyB.m_invMass;
    m_invIA = m_bodyA.m_invI;
    m_invIB = m_bodyB.m_invI;

    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;

    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();

    qA.set(aA);
    qB.set(aB);

    // use m_u as temporary variable
    Rot.mulToOutUnsafe(qA, m_u.setVec(m_localAnchorA).subLocal(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(qB, m_u.setVec(m_localAnchorB).subLocal(m_localCenterB), m_rB);
    m_u.setVec(cB).addLocalVec(m_rB).subLocal(cA).subLocal(m_rA);

    pool.pushRot(2);

    // Handle singularity.
    var length : Float = m_u.length();
    if (length > Settings.linearSlop) {
      m_u.x *= 1.0 / length;
      m_u.y *= 1.0 / length;
    } else {
      m_u.set(0.0, 0.0);
    }


    var crAu : Float = Vec2.crossVec(m_rA, m_u);
    var crBu : Float = Vec2.crossVec(m_rB, m_u);
    var invMass : Float = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

    // Compute the effective mass matrix.
    m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (m_frequencyHz > 0.0) {
      var C : Float = length - m_length;

      // Frequency
      var omega : Float = 2.0 * MathUtils.PI * m_frequencyHz;

      // Damping coefficient
      var d : Float = 2.0 * m_mass * m_dampingRatio * omega;

      // Spring stiffness
      var k : Float = m_mass * omega * omega;

      // magic formulas
      var h : Float = data.step.dt;
      m_gamma = h * (d + h * k);
      m_gamma = m_gamma != 0.0 ? 1.0 / m_gamma : 0.0;
      m_bias = C * h * k * m_gamma;

      invMass += m_gamma;
      m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      m_gamma = 0.0;
      m_bias = 0.0;
    }
    if (data.step.warmStarting) {

      // Scale the impulse to support a variable time step.
      m_impulse *= data.step.dtRatio;

      var P : Vec2 = pool.popVec2();
      P.setVec(m_u).mulLocal(m_impulse);

      vA.x -= m_invMassA * P.x;
      vA.y -= m_invMassA * P.y;
      wA -= m_invIA * Vec2.crossVec(m_rA, P);

      vB.x += m_invMassB * P.x;
      vB.y += m_invMassB * P.y;
      wB += m_invIB * Vec2.crossVec(m_rB, P);

      pool.pushVec2(1);
    } else {
      m_impulse = 0.0;
    }
//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var vpA : Vec2 = pool.popVec2();
    var vpB : Vec2 = pool.popVec2();

    // Cdot = dot(u, v + cross(w, r))
    Vec2.crossToOutUnsafeFVV(wA, m_rA, vpA);
    vpA.addLocalVec(vA);
    Vec2.crossToOutUnsafeFVV(wB, m_rB, vpB);
    vpB.addLocalVec(vB);
    var Cdot : Float = Vec2.dot(m_u, vpB.subLocal(vpA));

    var impulse : Float = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
    m_impulse += impulse;


    var Px : Float = impulse * m_u.x;
    var Py : Float = impulse * m_u.y;

    vA.x -= m_invMassA * Px;
    vA.y -= m_invMassA * Py;
    wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);
    vB.x += m_invMassB * Px;
    vB.y += m_invMassB * Py;
    wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(2);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    if (m_frequencyHz > 0.0) {
      return true;
    }
    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();
    var u : Vec2 = pool.popVec2();

    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;

    qA.set(aA);
    qB.set(aB);

    Rot.mulToOutUnsafe(qA, u.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, u.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    u.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);


    var length : Float = u.normalize();
    var C : Float = length - m_length;
    C = MathUtils.clamp(C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

    var impulse : Float = -m_mass * C;
    var Px : Float = impulse * u.x;
    var Py : Float = impulse * u.y;

    cA.x -= m_invMassA * Px;
    cA.y -= m_invMassA * Py;
    aA -= m_invIA * (rA.x * Py - rA.y * Px);
    cB.x += m_invMassB * Px;
    cB.y += m_invMassB * Py;
    aB += m_invIB * (rB.x * Py - rB.y * Px);

//    data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushVec2(3);
    pool.pushRot(2);

    return MathUtils.abs(C) < Settings.linearSlop;
  }
}

