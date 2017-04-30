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
 * Created at 3:38:38 AM Jan 15, 2011
 */
package box2d.dynamics.joints;

import box2d.common.Mat33;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.common.Vec3;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Angle constraint
//C = angle2 - angle1 - referenceAngle
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A weld joint essentially glues two bodies together. A weld joint may distort somewhat because the
 * island constraint solver is approximate.
 * 
 * @author Daniel Murphy
 */
 class WeldJoint extends Joint {

  private var m_frequencyHz : Float;
  private var m_dampingRatio : Float;
  private var m_bias : Float;

  // Solver shared
  private var m_localAnchorA : Vec2;
  private var m_localAnchorB : Vec2;
  private var m_referenceAngle : Float;
  private var m_gamma : Float;
  private var m_impulse : Vec3;


  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;
  private var m_mass : Mat33 = new Mat33();

  public function new(argWorld : IWorldPool, def : WeldJointDef) {
    super(argWorld, def);
    m_localAnchorA = new Vec2().setVec(def.localAnchorA);
    m_localAnchorB = new Vec2().setVec(def.localAnchorB);
    m_referenceAngle = def.referenceAngle;
    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;

    m_impulse = new Vec3();
    m_impulse.setZero();
  }
  
  public function getReferenceAngle() : Float {
    return m_referenceAngle;
  }

  public function getLocalAnchorA() : Vec2 {
    return m_localAnchorA;
  }

  public function getLocalAnchorB() : Vec2 {
    return m_localAnchorB;
  }

  public function getFrequency() : Float {
    return m_frequencyHz;
  }

  public function setFrequency(frequencyHz : Float) : Void {
    this.m_frequencyHz = frequencyHz;
  }

  public function getDampingRatio() : Float {
    return m_dampingRatio;
  }

  public function setDampingRatio(dampingRatio : Float) : Void {
    this.m_dampingRatio = dampingRatio;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.set(m_impulse.x, m_impulse.y);
    argOut.mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return inv_dt * m_impulse.z;
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

    var K : Mat33 = pool.popMat33();

    K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
    K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
    K.ez.x = -m_rA.y * iA - m_rB.y * iB;
    K.ex.y = K.ey.x;
    K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
    K.ez.y = m_rA.x * iA + m_rB.x * iB;
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = iA + iB;

    if (m_frequencyHz > 0.0) {
      K.getInverse22(m_mass);

      var invM : Float = iA + iB;
      var m : Float = invM > 0.0 ? 1.0 / invM : 0.0;

      var C : Float = aB - aA - m_referenceAngle;

      // Frequency
      var omega : Float = 2.0 * MathUtils.PI * m_frequencyHz;

      // Damping coefficient
      var d : Float = 2.0 * m * m_dampingRatio * omega;

      // Spring stiffness
      var k : Float = m * omega * omega;

      // magic formulas
      var h : Float = data.step.dt;
      m_gamma = h * (d + h * k);
      m_gamma = m_gamma != 0.0 ? 1.0 / m_gamma : 0.0;
      m_bias = C * h * k * m_gamma;

      invM += m_gamma;
      m_mass.ez.z = invM != 0.0 ? 1.0 / invM : 0.0;
    } else {
      K.getSymInverse33(m_mass);
      m_gamma = 0.0;
      m_bias = 0.0;
    }

    if (data.step.warmStarting) {
      var P : Vec2 = pool.popVec2();
      // Scale impulses to support a variable time step.
      m_impulse.mulLocal(data.step.dtRatio);

      P.set(m_impulse.x, m_impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.crossVec(m_rA, P) + m_impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.crossVec(m_rB, P) + m_impulse.z);
      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
    }

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
    pool.pushMat33(1);
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var Cdot1 : Vec2 = pool.popVec2();
    var P : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();
    if (m_frequencyHz > 0.0) {
      var Cdot2 : Float = wB - wA;

      var impulse2 : Float = -m_mass.ez.z * (Cdot2 + m_bias + m_gamma * m_impulse.z);
      m_impulse.z += impulse2;

      wA -= iA * impulse2;
      wB += iB * impulse2;

      Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot1);
      Vec2.crossToOutUnsafeFVV(wA, m_rA, temp);
      Cdot1.addLocalVec(vB).subLocal(vA).subLocal(temp);

      var impulse1 : Vec2 = P;
      Mat33.mul22ToOutUnsafe(m_mass, Cdot1, impulse1);
      impulse1.negateLocal();

      m_impulse.x += impulse1.x;
      m_impulse.y += impulse1.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * Vec2.crossVec(m_rA, P);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * Vec2.crossVec(m_rB, P);
    } else {
      Vec2.crossToOutUnsafeFVV(wA, m_rA, temp);
      Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot1);
      Cdot1.addLocalVec(vB).subLocal(vA).subLocal(temp);
      var Cdot2 : Float = wB - wA;

      var Cdot : Vec3 = pool.popVec3();
      Cdot.set(Cdot1.x, Cdot1.y, Cdot2);

      var impulse : Vec3 = pool.popVec3();
      Mat33.mulToOutUnsafe(m_mass, Cdot, impulse);
      impulse.negateLocal();
      m_impulse.addLocal(impulse);

      P.set(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.crossVec(m_rA, P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.crossVec(m_rB, P) + impulse.z);

      pool.pushVec3(2);
    }

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(3);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;
    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();
    var temp : Vec2 = pool.popVec2();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();

    qA.set(aA);
    qB.set(aB);

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    var positionError : Float;
    var angularError : Float;

    var K : Mat33 = pool.popMat33();
    var C1 : Vec2 = pool.popVec2();
    var P : Vec2 = pool.popVec2();

    K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    K.ez.x = -rA.y * iA - rB.y * iB;
    K.ex.y = K.ey.x;
    K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    K.ez.y = rA.x * iA + rB.x * iB;
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = iA + iB;
    if (m_frequencyHz > 0.0) {
      C1.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);

      positionError = C1.length();
      angularError = 0.0;

      K.solve22ToOut(C1, P);
      P.negateLocal();

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * Vec2.crossVec(rA, P);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * Vec2.crossVec(rB, P);
    } else {
      C1.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);
      var C2 : Float = aB - aA - m_referenceAngle;

      positionError = C1.length();
      angularError = MathUtils.abs(C2);

      var C : Vec3 = pool.popVec3();
      var impulse : Vec3 = pool.popVec3();
      C.set(C1.x, C1.y, C2);

      K.solve33ToOut(C, impulse);
      impulse.negateLocal();
      P.set(impulse.x, impulse.y);

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * (Vec2.crossVec(rA, P) + impulse.z);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * (Vec2.crossVec(rB, P) + impulse.z);
      pool.pushVec3(2);
    }

//    data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushVec2(5);
    pool.pushRot(2);
    pool.pushMat33(1);

    return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
  }
}

