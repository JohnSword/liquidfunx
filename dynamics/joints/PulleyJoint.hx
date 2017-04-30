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
/**
 * Created at 12:12:02 PM Jan 23, 2011
 */
package box2d.dynamics.joints;

import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

/**
 * The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
 * ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by
 * the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
 * when combined with prismatic joints. You should also cover the the anchor points with static
 * shapes to prevent one side from going to zero length.
 * 
 * @author Daniel Murphy
 */
 class PulleyJoint extends Joint {

  public static var MIN_PULLEY_LENGTH : Float = 2.0;

  private var m_groundAnchorA : Vec2 = new Vec2();
  private var m_groundAnchorB : Vec2 = new Vec2();
  private var m_lengthA : Float;
  private var m_lengthB : Float;

  // Solver shared
  private var m_localAnchorA : Vec2 = new Vec2();
  private var m_localAnchorB : Vec2 = new Vec2();
  private var m_constant : Float;
  private var m_ratio : Float;
  private var m_impulse : Float;

  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_uA : Vec2 = new Vec2();
  private var m_uB : Vec2 = new Vec2();
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;
  private var m_mass : Float;

  public function new(argWorldPool : IWorldPool, def : PulleyJointDef) {
    super(argWorldPool, def);
    m_groundAnchorA.setVec(def.groundAnchorA);
    m_groundAnchorB.setVec(def.groundAnchorB);
    m_localAnchorA.setVec(def.localAnchorA);
    m_localAnchorB.setVec(def.localAnchorB);

    m_ratio = def.ratio;

    m_lengthA = def.lengthA;
    m_lengthB = def.lengthB;

    m_constant = def.lengthA + m_ratio * def.lengthB;
    m_impulse = 0.0;
  }

  public function getLengthA() : Float {
    return m_lengthA;
  }

  public function getLengthB() : Float {
    return m_lengthB;
  }

  public function getCurrentLengthA() : Float {
    var p : Vec2 = pool.popVec2();
    m_bodyA.getWorldPointToOut(m_localAnchorA, p);
    p.subLocal(m_groundAnchorA);
    var length : Float = p.length();
    pool.pushVec2(1);
    return length;
  }

  public function getCurrentLengthB() : Float {
    var p : Vec2 = pool.popVec2();
    m_bodyB.getWorldPointToOut(m_localAnchorB, p);
    p.subLocal(m_groundAnchorB);
    var length : Float = p.length();
    pool.pushVec2(1);
    return length;
  }


  public function getLocalAnchorA() : Vec2 {
    return m_localAnchorA;
  }

  public function getLocalAnchorB() : Vec2 {
    return m_localAnchorB;
  }


  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.setVec(m_uB).mulLocal(m_impulse).mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return 0;
  }

  public function getGroundAnchorA() : Vec2 {
    return m_groundAnchorA;
  }

  public function getGroundAnchorB() : Vec2 {
    return m_groundAnchorB;
  }

  public function getLength1() : Float {
    var p : Vec2 = pool.popVec2();
    m_bodyA.getWorldPointToOut(m_localAnchorA, p);
    p.subLocal(m_groundAnchorA);

    var len : Float = p.length();
    pool.pushVec2(1);
    return len;
  }

  public function getLength2() : Float {
    var p : Vec2 = pool.popVec2();
    m_bodyB.getWorldPointToOut(m_localAnchorB, p);
    p.subLocal(m_groundAnchorB);

    var len : Float = p.length();
    pool.pushVec2(1);
    return len;
  }

  public function getRatio() : Float {
    return m_ratio;
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
    var temp : Vec2 = pool.popVec2();

    qA.set(aA);
    qB.set(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), m_rB);

    m_uA.setVec(cA).addLocalVec(m_rA).subLocal(m_groundAnchorA);
    m_uB.setVec(cB).addLocalVec(m_rB).subLocal(m_groundAnchorB);

    var lengthA : Float = m_uA.length();
    var lengthB : Float = m_uB.length();

    if (lengthA > 10 * Settings.linearSlop) {
      m_uA.mulLocal(1.0 / lengthA);
    } else {
      m_uA.setZero();
    }

    if (lengthB > 10 * Settings.linearSlop) {
      m_uB.mulLocal(1.0 / lengthB);
    } else {
      m_uB.setZero();
    }

    // Compute effective mass.
    var ruA : Float = Vec2.crossVec(m_rA, m_uA);
    var ruB : Float = Vec2.crossVec(m_rB, m_uB);

    var mA : Float = m_invMassA + m_invIA * ruA * ruA;
    var mB : Float = m_invMassB + m_invIB * ruB * ruB;

    m_mass = mA + m_ratio * m_ratio * mB;

    if (m_mass > 0.0) {
      m_mass = 1.0 / m_mass;
    }

    if (data.step.warmStarting) {

      // Scale impulses to support variable time steps.
      m_impulse *= data.step.dtRatio;

      // Warm starting.
      var PA : Vec2 = pool.popVec2();
      var PB : Vec2 = pool.popVec2();

      PA.setVec(m_uA).mulLocal(-m_impulse);
      PB.setVec(m_uB).mulLocal(-m_ratio * m_impulse);

      vA.x += m_invMassA * PA.x;
      vA.y += m_invMassA * PA.y;
      wA += m_invIA * Vec2.crossVec(m_rA, PA);
      vB.x += m_invMassB * PB.x;
      vB.y += m_invMassB * PB.y;
      wB += m_invIB * Vec2.crossVec(m_rB, PB);

      pool.pushVec2(2);
    } else {
      m_impulse = 0.0;
    }
//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var vpA : Vec2 = pool.popVec2();
    var vpB : Vec2 = pool.popVec2();
    var PA : Vec2 = pool.popVec2();
    var PB : Vec2 = pool.popVec2();

    Vec2.crossToOutUnsafeFVV(wA, m_rA, vpA);
    vpA.addLocalVec(vA);
    Vec2.crossToOutUnsafeFVV(wB, m_rB, vpB);
    vpB.addLocalVec(vB);

    var Cdot : Float = -Vec2.dot(m_uA, vpA) - m_ratio * Vec2.dot(m_uB, vpB);
    var impulse : Float = -m_mass * Cdot;
    m_impulse += impulse;

    PA.setVec(m_uA).mulLocal(-impulse);
    PB.setVec(m_uB).mulLocal(-m_ratio * impulse);
    vA.x += m_invMassA * PA.x;
    vA.y += m_invMassA * PA.y;
    wA += m_invIA * Vec2.crossVec(m_rA, PA);
    vB.x += m_invMassB * PB.x;
    vB.y += m_invMassB * PB.y;
    wB += m_invIB * Vec2.crossVec(m_rB, PB);

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(4);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();
    var uA : Vec2 = pool.popVec2();
    var uB : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();
    var PA : Vec2 = pool.popVec2();
    var PB : Vec2 = pool.popVec2();

    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;

    qA.set(aA);
    qB.set(aB);

    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);

    uA.setVec(cA).addLocalVec(rA).subLocal(m_groundAnchorA);
    uB.setVec(cB).addLocalVec(rB).subLocal(m_groundAnchorB);

    var lengthA : Float = uA.length();
    var lengthB : Float = uB.length();

    if (lengthA > 10.0 * Settings.linearSlop) {
      uA.mulLocal(1.0 / lengthA);
    } else {
      uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      uB.mulLocal(1.0 / lengthB);
    } else {
      uB.setZero();
    }

    // Compute effective mass.
    var ruA : Float = Vec2.crossVec(rA, uA);
    var ruB : Float = Vec2.crossVec(rB, uB);

    var mA : Float = m_invMassA + m_invIA * ruA * ruA;
    var mB : Float = m_invMassB + m_invIB * ruB * ruB;

    var mass : Float = mA + m_ratio * m_ratio * mB;

    if (mass > 0.0) {
      mass = 1.0 / mass;
    }

    var C : Float = m_constant - lengthA - m_ratio * lengthB;
    var linearError : Float = MathUtils.abs(C);

    var impulse : Float = -mass * C;

    PA.setVec(uA).mulLocal(-impulse);
    PB.setVec(uB).mulLocal(-m_ratio * impulse);

    cA.x += m_invMassA * PA.x;
    cA.y += m_invMassA * PA.y;
    aA += m_invIA * Vec2.crossVec(rA, PA);
    cB.x += m_invMassB * PB.x;
    cB.y += m_invMassB * PB.y;
    aB += m_invIB * Vec2.crossVec(rB, PB);

//    data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushRot(2);
    pool.pushVec2(7);

    return linearError < Settings.linearSlop;
  }
}

