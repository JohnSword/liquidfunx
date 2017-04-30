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

//Linear constraint (point-to-line)
//d = p2 - p1 = x2 + r2 - x1 - r1
//C = dot(perp, d)
//Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//   = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
//J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
//Angular constraint
//C = a2 - a1 + a_initial
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//
//K = J * invM * JT
//
//J = [-a -s1 a s2]
//  [0  -1  0  1]
//a = perp
//s1 = cross(d + r1, a) = cross(p2 - x1, a)
//s2 = cross(r2, a) = cross(p2 - x2, a)


//Motor/Limit linear constraint
//C = dot(ax1, d)
//Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
//J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

//Block Solver
//We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
//when the mass has poor distribution (leading to large torques about the joint anchor points).
//
//The Jacobian has 3 rows:
//J = [-uT -s1 uT s2] // linear
//  [0   -1   0  1] // angular
//  [-vT -a1 vT a2] // limit
//
//u = perp
//v = axis
//s1 = cross(d + r1, u), s2 = cross(r2, u)
//a1 = cross(d + r1, v), a2 = cross(r2, v)

//M * (v2 - v1) = JT * df
//J * v2 = bias
//
//v2 = v1 + invM * JT * df
//J * (v1 + invM * JT * df) = bias
//K * df = bias - J * v1 = -Cdot
//K = J * invM * JT
//Cdot = J * v1 - bias
//
//Now solve for f2.
//df = f2 - f1
//K * (f2 - f1) = -Cdot
//f2 = invK * (-Cdot) + f1
//
//Clamp accumulated limit impulse.
//lower: f2(3) = max(f2(3), 0)
//upper: f2(3) = min(f2(3), 0)
//
//Solve for correct f2(1:2)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                    = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
//f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
//Now compute impulse to be applied:
//df = f2 - f1

/**
 * A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
 * bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
 * and a joint motor to drive the motion or to model joint friction.
 * 
 * @author Daniel
 */
 class PrismaticJoint extends Joint {

  // Solver shared
  public var m_localAnchorA : Vec2;
  public var m_localAnchorB : Vec2;
  public var m_localXAxisA : Vec2;
  public var m_localYAxisA : Vec2;
  public var m_referenceAngle : Float;
  private var m_impulse : Vec3;
  private var m_motorImpulse : Float;
  private var m_lowerTranslation : Float;
  private var m_upperTranslation : Float;
  private var m_maxMotorForce : Float;
  private var m_motorSpeed : Float;
  private var m_enableLimit : Bool;
  private var m_enableMotor : Bool;
  private var m_limitState : LimitState;

  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;
  private var m_perp : Vec2;
  private var m_axis : Vec2;
  private var m_s2 : Float;
  private var m_s1 : Float;
  private var m_a2 : Float;
  private var m_a1 : Float;
  private var m_K : Mat33;
  private var m_motorMass : Float; // effective mass for motor/limit translational constraint.

  public function new(argWorld : IWorldPool, def : PrismaticJointDef) {
    super(argWorld, def);
    m_localAnchorA = new Vec2().setVec(def.localAnchorA);
    m_localAnchorB = new Vec2().setVec(def.localAnchorB);
    m_localXAxisA = new Vec2().setVec(def.localAxisA);
    m_localXAxisA.normalize();
    m_localYAxisA = new Vec2();
    Vec2.crossToOutUnsafeFVV(1, m_localXAxisA, m_localYAxisA);
    m_referenceAngle = def.referenceAngle;

    m_impulse = new Vec3();
    m_motorMass = 0.0;
    m_motorImpulse = 0.0;

    m_lowerTranslation = def.lowerTranslation;
    m_upperTranslation = def.upperTranslation;
    m_maxMotorForce = def.maxMotorForce;
    m_motorSpeed = def.motorSpeed;
    m_enableLimit = def.enableLimit;
    m_enableMotor = def.enableMotor;
    m_limitState = LimitState.INACTIVE;

    m_K = new Mat33();
    m_axis = new Vec2();
    m_perp = new Vec2();
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
    temp.setVec(m_axis).mulLocal(m_motorImpulse + m_impulse.z);
    argOut.setVec(m_perp).mulLocal(m_impulse.x).addLocalVec(temp).mulLocal(inv_dt);
    pool.pushVec2(1);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return inv_dt * m_impulse.y;
  }

  /**
   * Get the current joint translation, usually in meters.
   */
  public function getJointSpeed() : Float {
    var bA : Body = m_bodyA;
    var bB : Body = m_bodyB;

    var temp : Vec2 = pool.popVec2();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();
    var p1 : Vec2 = pool.popVec2();
    var p2 : Vec2 = pool.popVec2();
    var d : Vec2 = pool.popVec2();
    var axis : Vec2 = pool.popVec2();
    var temp2 : Vec2 = pool.popVec2();
    var temp3 : Vec2 = pool.popVec2();

    temp.setVec(m_localAnchorA).subLocal(bA.m_sweep.localCenter);
    Rot.mulToOutUnsafe(bA.m_xf.q, temp, rA);

    temp.setVec(m_localAnchorB).subLocal(bB.m_sweep.localCenter);
    Rot.mulToOutUnsafe(bB.m_xf.q, temp, rB);

    p1.setVec(bA.m_sweep.c).addLocalVec(rA);
    p2.setVec(bB.m_sweep.c).addLocalVec(rB);

    d.setVec(p2).subLocal(p1);
    Rot.mulToOutUnsafe(bA.m_xf.q, m_localXAxisA, axis);

    var vA : Vec2 = bA.m_linearVelocity;
    var vB : Vec2 = bB.m_linearVelocity;
    var wA : Float = bA.m_angularVelocity;
    var wB : Float = bB.m_angularVelocity;


    Vec2.crossToOutUnsafeFVV(wA, axis, temp);
    Vec2.crossToOutUnsafeFVV(wB, rB, temp2);
    Vec2.crossToOutUnsafeFVV(wA, rA, temp3);

    temp2.addLocalVec(vB).subLocal(vA).subLocal(temp3);
    var speed : Float = Vec2.dot(d, temp) + Vec2.dot(axis, temp2);

    pool.pushVec2(9);

    return speed;
  }

  public function getJointTranslation() : Float {
    var pA : Vec2 = pool.popVec2(), pB = pool.popVec2(), axis = pool.popVec2();
    m_bodyA.getWorldPointToOut(m_localAnchorA, pA);
    m_bodyB.getWorldPointToOut(m_localAnchorB, pB);
    m_bodyA.getWorldVectorToOutUnsafe(m_localXAxisA, axis);
    pB.subLocal(pA);
    var translation : Float = Vec2.dot(pB, axis);
    pool.pushVec2(3);
    return translation;
  }

  /**
   * Is the joint limit enabled?
   * 
   * @return
   */
  public function isLimitEnabled() : Bool {
    return m_enableLimit;
  }

  /**
   * Enable/disable the joint limit.
   * 
   * @param flag
   */
  public function enableLimit(flag : Bool) : Void {
    if (flag != m_enableLimit) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_enableLimit = flag;
      m_impulse.z = 0.0;
    }
  }

  /**
   * Get the lower joint limit, usually in meters.
   * 
   * @return
   */
  public function getLowerLimit() : Float {
    return m_lowerTranslation;
  }

  /**
   * Get the upper joint limit, usually in meters.
   * 
   * @return
   */
  public function getUpperLimit() : Float {
    return m_upperTranslation;
  }

  /**
   * Set the joint limits, usually in meters.
   * 
   * @param lower
   * @param upper
   */
  public function setLimits(lower : Float, upper : Float) : Void {
    if (lower != m_lowerTranslation || upper != m_upperTranslation) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_lowerTranslation = lower;
      m_upperTranslation = upper;
      m_impulse.z = 0.0;
    }
  }

  /**
   * Is the joint motor enabled?
   * 
   * @return
   */
  public function isMotorEnabled() : Bool {
    return m_enableMotor;
  }

  /**
   * Enable/disable the joint motor.
   * 
   * @param flag
   */
  public function enableMotor(flag : Bool) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  /**
   * Set the motor speed, usually in meters per second.
   * 
   * @param speed
   */
  public function setMotorSpeed(speed : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  /**
   * Get the motor speed, usually in meters per second.
   * 
   * @return
   */
  public function getMotorSpeed() : Float {
    return m_motorSpeed;
  }

  /**
   * Set the maximum motor force, usually in N.
   * 
   * @param force
   */
  public function setMaxMotorForce(force : Float) : Void {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorForce = force;
  }

  /**
   * Get the current motor force, usually in N.
   * 
   * @param inv_dt
   * @return
   */
  public function getMotorForce(inv_dt : Float) : Float {
    return m_motorImpulse * inv_dt;
  }

  public function getMaxMotorForce() : Float {
    return m_maxMotorForce;
  }

  public function getReferenceAngle() : Float {
    return m_referenceAngle;
  }

  public function getLocalAxisA() : Vec2 {
    return m_localXAxisA;
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
    var d : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();

    qA.set(aA);
    qB.set(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, d.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, d.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.setVec(cB).subLocal(cA).addLocalVec(rB).subLocal(rA);

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    // Compute motor Jacobian and effective mass.
    {
      Rot.mulToOutUnsafe(qA, m_localXAxisA, m_axis);
      temp.setVec(d).addLocalVec(rA);
      m_a1 = Vec2.crossVec(temp, m_axis);
      m_a2 = Vec2.crossVec(rB, m_axis);

      m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
      if (m_motorMass > 0.0) {
        m_motorMass = 1.0 / m_motorMass;
      }
    }

    // Prismatic constraint.
    {
      Rot.mulToOutUnsafe(qA, m_localYAxisA, m_perp);

      temp.setVec(d).addLocalVec(rA);
      m_s1 = Vec2.crossVec(temp, m_perp);
      m_s2 = Vec2.crossVec(rB, m_perp);

      var k11 : Float = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
      var k12 : Float = iA * m_s1 + iB * m_s2;
      var k13 : Float = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
      var k22 : Float = iA + iB;
      if (k22 == 0.0) {
        // For bodies with fixed rotation.
        k22 = 1.0;
      }
      var k23 : Float = iA * m_a1 + iB * m_a2;
      var k33 : Float = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

      m_K.ex.set(k11, k12, k13);
      m_K.ey.set(k12, k22, k23);
      m_K.ez.set(k13, k23, k33);
    }

    // Compute motor and limit terms.
    if (m_enableLimit) {

      var jointTranslation : Float = Vec2.dot(m_axis, d);
      if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0 * Settings.linearSlop) {
        m_limitState = LimitState.EQUAL;
      } else if (jointTranslation <= m_lowerTranslation) {
        if (m_limitState != LimitState.AT_LOWER) {
          m_limitState = LimitState.AT_LOWER;
          m_impulse.z = 0.0;
        }
      } else if (jointTranslation >= m_upperTranslation) {
        if (m_limitState != LimitState.AT_UPPER) {
          m_limitState = LimitState.AT_UPPER;
          m_impulse.z = 0.0;
        }
      } else {
        m_limitState = LimitState.INACTIVE;
        m_impulse.z = 0.0;
      }
    } else {
      m_limitState = LimitState.INACTIVE;
      m_impulse.z = 0.0;
    }

    if (m_enableMotor == false) {
      m_motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      m_impulse.mulLocal(data.step.dtRatio);
      m_motorImpulse *= data.step.dtRatio;

      var P : Vec2 = pool.popVec2();
      temp.setVec(m_axis).mulLocal(m_motorImpulse + m_impulse.z);
      P.setVec(m_perp).mulLocal(m_impulse.x).addLocalVec(temp);

      var LA : Float = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
      var LB : Float = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
      m_motorImpulse = 0.0;
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(4);
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var temp : Vec2 = pool.popVec2();

    // Solve linear motor constraint.
    if (m_enableMotor && m_limitState != LimitState.EQUAL) {
      temp.setVec(vB).subLocal(vA);
      var Cdot : Float = Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;
      var impulse : Float = m_motorMass * (m_motorSpeed - Cdot);
      var oldImpulse : Float = m_motorImpulse;
      var maxImpulse : Float = data.step.dt * m_maxMotorForce;
      m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_motorImpulse - oldImpulse;

      var P : Vec2 = pool.popVec2();
      P.setVec(m_axis).mulLocal(impulse);
      var LA : Float = impulse * m_a1;
      var LB : Float = impulse * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    }

    var Cdot1 : Vec2 = pool.popVec2();
    temp.setVec(vB).subLocal(vA);
    Cdot1.x = Vec2.dot(m_perp, temp) + m_s2 * wB - m_s1 * wA;
    Cdot1.y = wB - wA;
    // System.out.println(Cdot1);

    if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
      // Solve prismatic and limit constraint in block form.
      var Cdot2 : Float;
      temp.setVec(vB).subLocal(vA);
      Cdot2 = Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;

      var Cdot : Vec3 = pool.popVec3();
      Cdot.set(Cdot1.x, Cdot1.y, Cdot2);

      var f1 : Vec3 = pool.popVec3();
      var df : Vec3 = pool.popVec3();

      f1.setVec(m_impulse);
      m_K.solve33ToOut(Cdot.negateLocal(), df);
      // Cdot.negateLocal(); not used anymore
      m_impulse.addLocal(df);

      if (m_limitState == LimitState.AT_LOWER) {
        m_impulse.z = MathUtils.max(m_impulse.z, 0.0);
      } else if (m_limitState == LimitState.AT_UPPER) {
        m_impulse.z = MathUtils.min(m_impulse.z, 0.0);
      }

      // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
      // f1(1:2)
      var b : Vec2 = pool.popVec2();
      var f2r : Vec2 = pool.popVec2();

      temp.set(m_K.ez.x, m_K.ez.y).mulLocal(m_impulse.z - f1.z);
      b.setVec(Cdot1).negateLocal().subLocal(temp);

      m_K.solve22ToOut(b, f2r);
      f2r.addLocal(f1.x, f1.y);
      m_impulse.x = f2r.x;
      m_impulse.y = f2r.y;

      df.setVec(m_impulse).subLocal(f1);

      var P : Vec2 = pool.popVec2();
      temp.setVec(m_axis).mulLocal(df.z);
      P.setVec(m_perp).mulLocal(df.x).addLocalVec(temp);

      var LA : Float = df.x * m_s1 + df.y + df.z * m_a1;
      var LB : Float = df.x * m_s2 + df.y + df.z * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(3);
      pool.pushVec3(3);
    } else {
      // Limit is inactive, just solve the prismatic constraint in block form.
      var df : Vec2 = pool.popVec2();
      m_K.solve22ToOut(Cdot1.negateLocal(), df);
      Cdot1.negateLocal();

      m_impulse.x += df.x;
      m_impulse.y += df.y;

      var P : Vec2 = pool.popVec2();
      P.setVec(m_perp).mulLocal(df.x);
      var LA : Float = df.x * m_s1 + df.y;
      var LB : Float = df.x * m_s2 + df.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(2);
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(2);
  }


  override public function solvePositionConstraints(data : SolverData) : Bool {

    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();
    var d : Vec2 = pool.popVec2();
    var axis : Vec2 = pool.popVec2();
    var perp : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();
    var C1 : Vec2 = pool.popVec2();

    var impulse : Vec3 = pool.popVec3();

    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;

    qA.set(aA);
    qB.set(aB);

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    // Compute fresh Jacobians
    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);

    Rot.mulToOutUnsafe(qA, m_localXAxisA, axis);
    var a1 : Float = Vec2.crossVec(temp.setVec(d).addLocalVec(rA), axis);
    var a2 : Float = Vec2.crossVec(rB, axis);
    Rot.mulToOutUnsafe(qA, m_localYAxisA, perp);

    var s1 : Float = Vec2.crossVec(temp.setVec(d).addLocalVec(rA), perp);
    var s2 : Float = Vec2.crossVec(rB, perp);

    C1.x = Vec2.dot(perp, d);
    C1.y = aB - aA - m_referenceAngle;

    var linearError : Float = MathUtils.abs(C1.x);
    var angularError : Float = MathUtils.abs(C1.y);

    var active : Bool = false;
    var C2 : Float = 0.0;
    if (m_enableLimit) {
      var translation : Float = Vec2.dot(axis, d);
      if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0 * Settings.linearSlop) {
        // Prevent large angular corrections
        C2 =
            MathUtils.clamp(translation, -Settings.maxLinearCorrection,
                Settings.maxLinearCorrection);
        linearError = MathUtils.max(linearError, MathUtils.abs(translation));
        active = true;
      } else if (translation <= m_lowerTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 =
            MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop,
                -Settings.maxLinearCorrection, 0.0);
        linearError = MathUtils.max(linearError, m_lowerTranslation - translation);
        active = true;
      } else if (translation >= m_upperTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 =
            MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0,
                Settings.maxLinearCorrection);
        linearError = MathUtils.max(linearError, translation - m_upperTranslation);
        active = true;
      }
    }

    if (active) {
      var k11 : Float = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      var k12 : Float = iA * s1 + iB * s2;
      var k13 : Float = iA * s1 * a1 + iB * s2 * a2;
      var k22 : Float = iA + iB;
      if (k22 == 0.0) {
        // For fixed rotation
        k22 = 1.0;
      }
      var k23 : Float = iA * a1 + iB * a2;
      var k33 : Float = mA + mB + iA * a1 * a1 + iB * a2 * a2;

      var K : Mat33 = pool.popMat33();
      K.ex.set(k11, k12, k13);
      K.ey.set(k12, k22, k23);
      K.ez.set(k13, k23, k33);

      var C : Vec3 = pool.popVec3();
      C.x = C1.x;
      C.y = C1.y;
      C.z = C2;

      K.solve33ToOut(C.negateLocal(), impulse);
      pool.pushVec3(1);
      pool.pushMat33(1);
    } else {
      var k11 : Float = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      var k12 : Float = iA * s1 + iB * s2;
      var k22 : Float = iA + iB;
      if (k22 == 0.0) {
        k22 = 1.0;
      }

      var K : Mat22 = pool.popMat22();
      K.ex.set(k11, k12);
      K.ey.set(k12, k22);

      // temp is impulse1
      K.solveToOut(C1.negateLocal(), temp);
      C1.negateLocal();

      impulse.x = temp.x;
      impulse.y = temp.y;
      impulse.z = 0.0;

      pool.pushMat22(1);
    }

    var Px : Float = impulse.x * perp.x + impulse.z * axis.x;
    var Py : Float = impulse.x * perp.y + impulse.z * axis.y;
    var LA : Float = impulse.x * s1 + impulse.y + impulse.z * a1;
    var LB : Float = impulse.x * s2 + impulse.y + impulse.z * a2;

    cA.x -= mA * Px;
    cA.y -= mA * Py;
    aA -= iA * LA;
    cB.x += mB * Px;
    cB.y += mB * Py;
    aB += iB * LB;

    // data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushVec2(7);
    pool.pushVec3(1);
    pool.pushRot(2);

    return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
  }
}

