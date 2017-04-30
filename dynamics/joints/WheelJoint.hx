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
package box2d.dynamics.joints;

import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//Linear constraint (point-to-line)
//d = pB - pA = xB + rB - xA - rA
//C = dot(ay, d)
//Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//   = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
//J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

//Spring linear constraint
//C = dot(ax, d)
//Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
//J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

//Motor rotational constraint
//Cdot = wB - wA
//J = [0 0 -1 0 0 1]

/**
 * A wheel joint. This joint provides two degrees of freedom: translation along an axis fixed in
 * bodyA and rotation in the plane. You can use a joint limit to restrict the range of motion and a
 * joint motor to drive the rotation or to model rotational friction. This joint is designed for
 * vehicle suspensions.
 * 
 * @author Daniel Murphy
 */
 class WheelJoint extends Joint {

  private var m_frequencyHz : Float;
  private var m_dampingRatio : Float;

  // Solver shared
  private var m_localAnchorA : Vec2 = new Vec2();
  private var m_localAnchorB : Vec2 = new Vec2();
  private var m_localXAxisA : Vec2 = new Vec2();
  private var m_localYAxisA : Vec2 = new Vec2();

  private var m_impulse : Float;
  private var m_motorImpulse : Float;
  private var m_springImpulse : Float;

  private var m_maxMotorTorque : Float;
  private var m_motorSpeed : Float;
  private var m_enableMotor : Bool;

  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;

  private var m_ax : Vec2 = new Vec2();
  private var m_ay : Vec2 = new Vec2();
  private var m_sBx : Float;
  private var m_sAx : Float;
  private var m_sBy : Float;
  private var m_sAy : Float;

  private var m_mass : Float;
  private var m_motorMass : Float;
  private var m_springMass : Float;

  private var m_bias : Float;
  private var m_gamma : Float;

  public function new(argPool : IWorldPool, def : WheelJointDef) {
    super(argPool, def);
    m_localAnchorA.setVec(def.localAnchorA);
    m_localAnchorB.setVec(def.localAnchorB);
    m_localXAxisA.setVec(def.localAxisA);
    Vec2.crossToOutUnsafeFVV(1.0, m_localXAxisA, m_localYAxisA);


    m_motorMass = 0.0;
    m_motorImpulse = 0.0;

    m_maxMotorTorque = def.maxMotorTorque;
    m_motorSpeed = def.motorSpeed;
    m_enableMotor = def.enableMotor;

    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;
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
    var temp : Vec2 = pool.popVec2();
    temp.setVec(m_ay).mulLocal(m_impulse);
    argOut.setVec(m_ax).mulLocal(m_springImpulse).addLocalVec(temp).mulLocal(inv_dt);
    pool.pushVec2(1);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return inv_dt * m_motorImpulse;
  }

  public function getJointTranslation() : Float {
    var b1 : Body = m_bodyA;
    var b2 : Body = m_bodyB;

    var p1 : Vec2 = pool.popVec2();
    var p2 : Vec2 = pool.popVec2();
    var axis : Vec2 = pool.popVec2();
    b1.getWorldPointToOut(m_localAnchorA, p1);
    b2.getWorldPointToOut(m_localAnchorA, p2);
    p2.subLocal(p1);
    b1.getWorldVectorToOut(m_localXAxisA, axis);

    var translation : Float = Vec2.dot(p2, axis);
    pool.pushVec2(3);
    return translation;
  }

  /** For serialization */
  public function getLocalAxisA() : Vec2 {
    return m_localXAxisA;
  }

  public function getJointSpeed() : Float {
    return m_bodyA.m_angularVelocity - m_bodyB.m_angularVelocity;
  }

  public function isMotorEnabled() : Bool {
    return m_enableMotor;
  }

  public function enableMotor(flag : Bool) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  public function setMotorSpeed(speed : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  public function getMotorSpeed() : Float {
    return m_motorSpeed;
  }

  public function getMaxMotorTorque() : Float {
    return m_maxMotorTorque;
  }

  public function setMaxMotorTorque(torque : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorTorque = torque;
  }

  public function getMotorTorque(inv_dt : Float) : Float {
    return m_motorImpulse * inv_dt;
  }

  public function setSpringFrequencyHz(hz : Float) : Void {
    m_frequencyHz = hz;
  }

  public function getSpringFrequencyHz() : Float {
    return m_frequencyHz;
  }

  public function setSpringDampingRatio(ratio : Float) : Void {
    m_dampingRatio = ratio;
  }

  public function getSpringDampingRatio() : Float {
    return m_dampingRatio;
  }

  // pooling
  private var rA : Vec2 = new Vec2();
  private var rB : Vec2 = new Vec2();
  private var d : Vec2 = new Vec2();

  override public function initVelocityConstraints(data : SolverData) : Void {
    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterA.setVec(m_bodyA.m_sweep.localCenter);
    m_localCenterB.setVec(m_bodyB.m_sweep.localCenter);
    m_invMassA = m_bodyA.m_invMass;
    m_invMassB = m_bodyB.m_invMass;
    m_invIA = m_bodyA.m_invI;
    m_invIB = m_bodyB.m_invI;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

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
    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);

    // Point to line constraint
    {
      Rot.mulToOut(qA, m_localYAxisA, m_ay);
      m_sAy = Vec2.crossVec(temp.setVec(d).addLocalVec(rA), m_ay);
      m_sBy = Vec2.crossVec(rB, m_ay);

      m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

      if (m_mass > 0.0) {
        m_mass = 1.0 / m_mass;
      }
    }

    // Spring constraint
    m_springMass = 0.0;
    m_bias = 0.0;
    m_gamma = 0.0;
    if (m_frequencyHz > 0.0) {
      Rot.mulToOut(qA, m_localXAxisA, m_ax);
      m_sAx = Vec2.crossVec(temp.setVec(d).addLocalVec(rA), m_ax);
      m_sBx = Vec2.crossVec(rB, m_ax);

      var invMass : Float = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

      if (invMass > 0.0) {
        m_springMass = 1.0 / invMass;

        var C : Float = Vec2.dot(d, m_ax);

        // Frequency
        var omega : Float = 2.0 * MathUtils.PI * m_frequencyHz;

        // Damping coefficient
        var d : Float = 2.0 * m_springMass * m_dampingRatio * omega;

        // Spring stiffness
        var k : Float = m_springMass * omega * omega;

        // magic formulas
        var h : Float = data.step.dt;
        m_gamma = h * (d + h * k);
        if (m_gamma > 0.0) {
          m_gamma = 1.0 / m_gamma;
        }

        m_bias = C * h * k * m_gamma;

        m_springMass = invMass + m_gamma;
        if (m_springMass > 0.0) {
          m_springMass = 1.0 / m_springMass;
        }
      }
    } else {
      m_springImpulse = 0.0;
    }

    // Rotational motor
    if (m_enableMotor) {
      m_motorMass = iA + iB;
      if (m_motorMass > 0.0) {
        m_motorMass = 1.0 / m_motorMass;
      }
    } else {
      m_motorMass = 0.0;
      m_motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      var P : Vec2 = pool.popVec2();
      // Account for variable time step.
      m_impulse *= data.step.dtRatio;
      m_springImpulse *= data.step.dtRatio;
      m_motorImpulse *= data.step.dtRatio;

      P.x = m_impulse * m_ay.x + m_springImpulse * m_ax.x;
      P.y = m_impulse * m_ay.y + m_springImpulse * m_ax.y;
      var LA : Float = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
      var LB : Float = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

      vA.x -= m_invMassA * P.x;
      vA.y -= m_invMassA * P.y;
      wA -= m_invIA * LA;

      vB.x += m_invMassB * P.x;
      vB.y += m_invMassB * P.y;
      wB += m_invIB * LB;
      pool.pushVec2(1);
    } else {
      m_impulse = 0.0;
      m_springImpulse = 0.0;
      m_motorImpulse = 0.0;
    }
    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var temp : Vec2 = pool.popVec2();
    var P : Vec2 = pool.popVec2();

    // Solve spring constraint
    {
      var Cdot : Float = Vec2.dot(m_ax, temp.setVec(vB).subLocal(vA)) + m_sBx * wB - m_sAx * wA;
      var impulse : Float = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
      m_springImpulse += impulse;

      P.x = impulse * m_ax.x;
      P.y = impulse * m_ax.y;
      var LA : Float = impulse * m_sAx;
      var LB : Float = impulse * m_sBx;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
      var Cdot : Float = wB - wA - m_motorSpeed;
      var impulse : Float = -m_motorMass * Cdot;

      var oldImpulse : Float = m_motorImpulse;
      var maxImpulse : Float = data.step.dt * m_maxMotorTorque;
      m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve point to line constraint
    {
      var Cdot : Float = Vec2.dot(m_ay, temp.setVec(vB).subLocal(vA)) + m_sBy * wB - m_sAy * wA;
      var impulse : Float = -m_mass * Cdot;
      m_impulse += impulse;

      P.x = impulse * m_ay.x;
      P.y = impulse * m_ay.y;
      var LA : Float = impulse * m_sAy;
      var LB : Float = impulse * m_sBy;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }
    pool.pushVec2(2);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;

    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var temp : Vec2 = pool.popVec2();

    qA.set(aA);
    qB.set(aB);

    Rot.mulToOut(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOut(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.setVec(cB).subLocal(cA).addLocalVec(rB).subLocal(rA);

    var ay : Vec2 = pool.popVec2();
    Rot.mulToOut(qA, m_localYAxisA, ay);

    var sAy : Float = Vec2.crossVec(temp.setVec(d).addLocalVec(rA), ay);
    var sBy : Float = Vec2.crossVec(rB, ay);

    var C : Float = Vec2.dot(d, ay);

    var k : Float = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

    var impulse : Float;
    if (k != 0.0) {
      impulse = -C / k;
    } else {
      impulse = 0.0;
    }

    var P : Vec2 = pool.popVec2();
    P.x = impulse * ay.x;
    P.y = impulse * ay.y;
    var LA : Float = impulse * sAy;
    var LB : Float = impulse * sBy;

    cA.x -= m_invMassA * P.x;
    cA.y -= m_invMassA * P.y;
    aA -= m_invIA * LA;
    cB.x += m_invMassB * P.x;
    cB.y += m_invMassB * P.y;
    aB += m_invIB * LB;

    pool.pushVec2(3);
    pool.pushRot(2);
    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return MathUtils.abs(C) <= Settings.linearSlop;
  }
}

