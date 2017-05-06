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

import box2d.common.Mat22;
import box2d.common.Mat33;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.common.Vec3;
import box2d.dynamics.Body;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Motor constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A revolute joint constrains two bodies to share a common point while they are free to rotate
 * about the point. The relative rotation about the shared point is the joint angle. You can limit
 * the relative rotation with a joint limit that specifies a lower and upper angle. You can use a
 * motor to drive the relative rotation about the shared point. A maximum motor torque is provided
 * so that infinite forces are not generated.
 * 
 * @author Daniel Murphy
 */
 class RevoluteJoint extends Joint {

  // Solver shared
  public var m_localAnchorA : Vec2 = new Vec2();
  public var m_localAnchorB : Vec2 = new Vec2();
  private var m_impulse : Vec3 = new Vec3();
  private var m_motorImpulse : Float = 0;

  private var m_enableMotor : Bool;
  private var m_maxMotorTorque : Float = 0;
  private var m_motorSpeed : Float = 0;

  private var m_enableLimit : Bool;
  public var m_referenceAngle : Float = 0;
  private var m_lowerAngle : Float = 0;
  private var m_upperAngle : Float = 0;

  // Solver temp
  private var m_indexA : Int = 0;
  private var m_indexB : Int = 0;
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float = 0;
  private var m_invMassB : Float = 0;
  private var m_invIA : Float = 0;
  private var m_invIB : Float = 0;
  private var m_mass : Mat33 = new Mat33();
  private var m_motorMass : Float = 0; // effective mass for motor/limit angular constraint.
  private var m_limitState : LimitState;

  public function new(argWorld : IWorldPool, def : RevoluteJointDef) {
    super(argWorld, def);
    m_localAnchorA.setVec(def.localAnchorA);
    m_localAnchorB.setVec(def.localAnchorB);
    m_referenceAngle = def.referenceAngle;

    m_motorImpulse = 0;

    m_lowerAngle = def.lowerAngle;
    m_upperAngle = def.upperAngle;
    m_maxMotorTorque = def.maxMotorTorque;
    m_motorSpeed = def.motorSpeed;
    m_enableLimit = def.enableLimit;
    m_enableMotor = def.enableMotor;
    m_limitState = LimitState.INACTIVE;
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

    // Vec2 cA = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;

    // Vec2 cB = data.positions[m_indexB].c;
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

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var fixedRotation : Bool = (iA + iB == 0.0);

    m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
    m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
    m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
    m_mass.ex.y = m_mass.ey.x;
    m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
    m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
    m_mass.ex.z = m_mass.ez.x;
    m_mass.ey.z = m_mass.ez.y;
    m_mass.ez.z = iA + iB;

    m_motorMass = iA + iB;
    if (m_motorMass > 0.0) {
      m_motorMass = 1.0 / m_motorMass;
    }

    if (m_enableMotor == false || fixedRotation) {
      m_motorImpulse = 0.0;
    }

    if (m_enableLimit && fixedRotation == false) {
      var jointAngle : Float = aB - aA - m_referenceAngle;
      if (MathUtils.abs(m_upperAngle - m_lowerAngle) < 2.0 * Settings.angularSlop) {
        m_limitState = LimitState.EQUAL;
      } else if (jointAngle <= m_lowerAngle) {
        if (m_limitState != LimitState.AT_LOWER) {
          m_impulse.z = 0.0;
        }
        m_limitState = LimitState.AT_LOWER;
      } else if (jointAngle >= m_upperAngle) {
        if (m_limitState != LimitState.AT_UPPER) {
          m_impulse.z = 0.0;
        }
        m_limitState = LimitState.AT_UPPER;
      } else {
        m_limitState = LimitState.INACTIVE;
        m_impulse.z = 0.0;
      }
    } else {
      m_limitState = LimitState.INACTIVE;
    }

    if (data.step.warmStarting) {
      var P : Vec2 = pool.popVec2();
      // Scale impulses to support a variable time step.
      m_impulse.x *= data.step.dtRatio;
      m_impulse.y *= data.step.dtRatio;
      m_motorImpulse *= data.step.dtRatio;

      P.x = m_impulse.x;
      P.y = m_impulse.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.crossVec(m_rA, P) + m_motorImpulse + m_impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.crossVec(m_rB, P) + m_motorImpulse + m_impulse.z);
      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
      m_motorImpulse = 0.0;
    }
    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var fixedRotation : Bool = (iA + iB == 0.0);

    // Solve motor constraint.
    if (m_enableMotor && m_limitState != LimitState.EQUAL && fixedRotation == false) {
      var Cdot : Float = wB - wA - m_motorSpeed;
      var impulse : Float = -m_motorMass * Cdot;
      var oldImpulse : Float = m_motorImpulse;
      var maxImpulse : Float = data.step.dt * m_maxMotorTorque;
      m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }
    var temp : Vec2 = pool.popVec2();

    // Solve limit constraint.
    if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false) {

      var Cdot1 : Vec2 = pool.popVec2();
      var Cdot : Vec3 = pool.popVec3();

      // Solve point-to-point constraint
      Vec2.crossToOutUnsafeFVV(wA, m_rA, temp);
      Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot1);
      Cdot1.addLocalVec(vB).subLocal(vA).subLocal(temp);
      var Cdot2 : Float = wB - wA;
      Cdot.set(Cdot1.x, Cdot1.y, Cdot2);

      var impulse : Vec3 = pool.popVec3();
      m_mass.solve33ToOut(Cdot, impulse);
      impulse.negateLocal();

      if (m_limitState == LimitState.EQUAL) {
        m_impulse.addLocal(impulse);
      } else if (m_limitState == LimitState.AT_LOWER) {
        var newImpulse : Float = m_impulse.z + impulse.z;
        if (newImpulse < 0.0) {
          var rhs : Vec2 = pool.popVec2();
          rhs.set(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(Cdot1);
          m_mass.solve22ToOut(rhs, temp);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -m_impulse.z;
          m_impulse.x += temp.x;
          m_impulse.y += temp.y;
          m_impulse.z = 0.0;
          pool.pushVec2(1);
        } else {
          m_impulse.addLocal(impulse);
        }
      } else if (m_limitState == LimitState.AT_UPPER) {
        var newImpulse : Float = m_impulse.z + impulse.z;
        if (newImpulse > 0.0) {
          var rhs : Vec2 = pool.popVec2();
          rhs.set(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(Cdot1);
          m_mass.solve22ToOut(rhs, temp);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -m_impulse.z;
          m_impulse.x += temp.x;
          m_impulse.y += temp.y;
          m_impulse.z = 0.0;
          pool.pushVec2(1);
        } else {
          m_impulse.addLocal(impulse);
        }
      }
      var P : Vec2 = pool.popVec2();

      P.set(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.crossVec(m_rA, P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.crossVec(m_rB, P) + impulse.z);

      pool.pushVec2(2);
      pool.pushVec3(2);
    } else {

      // Solve point-to-point constraint
      var Cdot : Vec2 = pool.popVec2();
      var impulse : Vec2 = pool.popVec2();

      Vec2.crossToOutUnsafeFVV(wA, m_rA, temp);
      Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot);
      Cdot.addLocalVec(vB).subLocal(vA).subLocal(temp);
      m_mass.solve22ToOut(Cdot.negateLocal(), impulse); // just leave negated

      m_impulse.x += impulse.x;
      m_impulse.y += impulse.y;

      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * Vec2.crossVec(m_rA, impulse);

      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * Vec2.crossVec(m_rB, impulse);

      pool.pushVec2(2);
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;

    qA.set(aA);
    qB.set(aB);

    var angularError : Float = 0.0;
    var positionError : Float = 0.0;

    var fixedRotation : Bool = (m_invIA + m_invIB == 0.0);

    // Solve angular limit constraint.
    if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false) {
      var angle : Float = aB - aA - m_referenceAngle;
      var limitImpulse : Float = 0.0;

      if (m_limitState == LimitState.EQUAL) {
        // Prevent large angular corrections
        var C : Float =
            MathUtils.clamp(angle - m_lowerAngle, -Settings.maxAngularCorrection,
                Settings.maxAngularCorrection);
        limitImpulse = -m_motorMass * C;
        angularError = MathUtils.abs(C);
      } else if (m_limitState == LimitState.AT_LOWER) {
        var C : Float = angle - m_lowerAngle;
        angularError = -C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clamp(C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0);
        limitImpulse = -m_motorMass * C;
      } else if (m_limitState == LimitState.AT_UPPER) {
        var C : Float = angle - m_upperAngle;
        angularError = C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clamp(C - Settings.angularSlop, 0.0, Settings.maxAngularCorrection);
        limitImpulse = -m_motorMass * C;
      }

      aA -= m_invIA * limitImpulse;
      aB += m_invIB * limitImpulse;
    }
    // Solve point-to-point constraint.
    {
      qA.set(aA);
      qB.set(aB);

      var rA : Vec2 = pool.popVec2();
      var rB : Vec2 = pool.popVec2();
      var C : Vec2 = pool.popVec2();
      var impulse : Vec2 = pool.popVec2();

      Rot.mulToOutUnsafe(qA, C.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
      Rot.mulToOutUnsafe(qB, C.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
      C.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);
      positionError = C.length();

      var mA : Float = m_invMassA, mB = m_invMassB;
      var iA : Float = m_invIA, iB = m_invIB;

      var K : Mat22 = pool.popMat22();
      K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
      K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
      K.ey.x = K.ex.y;
      K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
      K.solveToOut(C, impulse);
      impulse.negateLocal();

      cA.x -= mA * impulse.x;
      cA.y -= mA * impulse.y;
      aA -= iA * Vec2.crossVec(rA, impulse);

      cB.x += mB * impulse.x;
      cB.y += mB * impulse.y;
      aB += iB * Vec2.crossVec(rB, impulse);

      pool.pushVec2(4);
      pool.pushMat22(1);
    }
    // data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushRot(2);

    return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
  }
  
  public function getLocalAnchorA() : Vec2 {
    return m_localAnchorA;
  }
  
  public function getLocalAnchorB() : Vec2 {
    return m_localAnchorB;
  }
  
  public function getReferenceAngle() : Float {
    return m_referenceAngle;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.set(m_impulse.x, m_impulse.y).mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return inv_dt * m_impulse.z;
  }

  public function getJointAngle() : Float {
    var b1 : Body = m_bodyA;
    var b2 : Body = m_bodyB;
    return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
  }

  public function getJointSpeed() : Float {
    var b1 : Body = m_bodyA;
    var b2 : Body = m_bodyB;
    return b2.m_angularVelocity - b1.m_angularVelocity;
  }

  public function isMotorEnabled() : Bool {
    return m_enableMotor;
  }

  public function enableMotor(flag : Bool) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  public function getMotorTorque(inv_dt : Float) : Float {
    return m_motorImpulse * inv_dt;
  }

  public function setMotorSpeed(speed : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  public function setMaxMotorTorque(torque : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorTorque = torque;
  }

  public function getMotorSpeed() : Float {
    return m_motorSpeed;
  }

  public function getMaxMotorTorque() : Float {
    return m_maxMotorTorque;
  }

  public function isLimitEnabled() : Bool {
    return m_enableLimit;
  }

  public function enableLimit(flag : Bool) : Void {
    if (flag != m_enableLimit) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_enableLimit = flag;
      m_impulse.z = 0.0;
    }
  }

  public function getLowerLimit() : Float {
    return m_lowerAngle;
  }

  public function getUpperLimit() : Float {
    return m_upperAngle;
  }

  public function setLimits(lower : Float, upper : Float) : Void {
    if (lower != m_lowerAngle || upper != m_upperAngle) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_impulse.z = 0.0;
      m_lowerAngle = lower;
      m_upperAngle = upper;
    }
  }
}

