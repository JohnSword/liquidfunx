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
 * Created at 11:34:45 AM Jan 23, 2011
 */
package box2d.dynamics.joints;

import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//Gear Joint:
//C0 = (coordinate1 + ratio * coordinate2)_initial
//C = (coordinate1 + ratio * coordinate2) - C0 = 0
//J = [J1 ratio * J2]
//K = J * invM * JT
//= J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
//Revolute:
//coordinate = rotation
//Cdot = angularVelocity
//J = [0 0 1]
//K = J * invM * JT = invI
//
//Prismatic:
//coordinate = dot(p - pg, ug)
//Cdot = dot(v + cross(w, r), ug)
//J = [ug cross(r, ug)]
//K = J * invM * JT = invMass + invI * cross(r, ug)^2

/**
 * A gear joint is used to connect two joints together. Either joint can be a revolute or prismatic
 * joint. You specify a gear ratio to bind the motions together: coordinate1 + ratio * coordinate2 =
 * constant The ratio can be negative or positive. If one joint is a revolute joint and the other
 * joint is a prismatic joint, then the ratio will have units of length or units of 1/length.
 * 
 * @warning The revolute and prismatic joints must be attached to fixed bodies (which must be body1
 *          on those joints).
 * @warning You have to manually destroy the gear joint if joint1 or joint2 is destroyed.
 * @author Daniel Murphy
 */
 class GearJoint extends Joint {

  private var m_joint1 : Joint;
  private var m_joint2 : Joint;

  private var m_typeA : JointType;
  private var m_typeB : JointType;

  // Body A is connected to body C
  // Body B is connected to body D
  private var m_bodyC : Body;
  private var m_bodyD : Body;

  // Solver shared
  private var m_localAnchorA : Vec2 = new Vec2();
  private var m_localAnchorB : Vec2 = new Vec2();
  private var m_localAnchorC : Vec2 = new Vec2();
  private var m_localAnchorD : Vec2 = new Vec2();

  private var m_localAxisC : Vec2 = new Vec2();
  private var m_localAxisD : Vec2 = new Vec2();

  private var m_referenceAngleA : Float;
  private var m_referenceAngleB : Float;

  private var m_constant : Float;
  private var m_ratio : Float;

  private var m_impulse : Float;

  // Solver temp
  private var m_indexD : Int;
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_indexC : Int;
  private var m_lcA : Vec2 = new Vec2();
  private var m_lcB : Vec2 = new Vec2();
  private var m_lcC : Vec2 = new Vec2();
  private var m_lcD : Vec2 = new Vec2();
  private var m_mD : Float;
  private var m_mA : Float;
  private var m_mB : Float;
  private var m_mC : Float;
  private var m_iD : Float;
  private var m_iA : Float;
  private var m_iB : Float;
  private var m_iC : Float;
  private var m_JvBD : Vec2 = new Vec2();
  private var m_JvAC : Vec2 = new Vec2();
  private var m_JwD : Float;
  private var m_JwA : Float;
  private var m_JwB : Float;
  private var m_JwC : Float;
  private var m_mass : Float;

  public function new(argWorldPool : IWorldPool, def : GearJointDef) {
    super(argWorldPool, def);

    m_joint1 = def.joint1;
    m_joint2 = def.joint2;

    m_typeA = m_joint1.getType();
    m_typeB = m_joint2.getType();


    var coordinateA : Float;
    var coordinateB : Float;

    // TODO_ERIN there might be some problem with the joint edges in Joint.

    m_bodyC = m_joint1.getBodyA();
    m_bodyA = m_joint1.getBodyB();

    // Get geometry of joint1
    var xfA : Transform = m_bodyA.m_xf;
    var aA : Float = m_bodyA.m_sweep.a;
    var xfC : Transform = m_bodyC.m_xf;
    var aC : Float = m_bodyC.m_sweep.a;

    if (m_typeA == JointType.REVOLUTE) {
      var revolute : RevoluteJoint = cast def.joint1;
      m_localAnchorC.setVec(revolute.m_localAnchorA);
      m_localAnchorA.setVec(revolute.m_localAnchorB);
      m_referenceAngleA = revolute.m_referenceAngle;
      m_localAxisC.setZero();

      coordinateA = aA - aC - m_referenceAngleA;
    } else {
      var pA : Vec2 = pool.popVec2();
      var temp : Vec2 = pool.popVec2();
      var prismatic : PrismaticJoint = cast def.joint1;
      m_localAnchorC.setVec(prismatic.m_localAnchorA);
      m_localAnchorA.setVec(prismatic.m_localAnchorB);
      m_referenceAngleA = prismatic.m_referenceAngle;
      m_localAxisC.setVec(prismatic.m_localXAxisA);

      var pC : Vec2 = m_localAnchorC;
      Rot.mulToOutUnsafe(xfA.q, m_localAnchorA, temp);
      temp.addLocalVec(xfA.p).subLocal(xfC.p);
      Rot.mulTransUnsafe2(xfC.q, temp, pA);
      coordinateA = Vec2.dot(pA.subLocal(pC), m_localAxisC);
      pool.pushVec2(2);
    }

    m_bodyD = m_joint2.getBodyA();
    m_bodyB = m_joint2.getBodyB();

    // Get geometry of joint2
    var xfB : Transform = m_bodyB.m_xf;
    var aB : Float = m_bodyB.m_sweep.a;
    var xfD : Transform = m_bodyD.m_xf;
    var aD : Float = m_bodyD.m_sweep.a;

    if (m_typeB == JointType.REVOLUTE) {
      var revolute : RevoluteJoint = cast def.joint2;
      m_localAnchorD.setVec(revolute.m_localAnchorA);
      m_localAnchorB.setVec(revolute.m_localAnchorB);
      m_referenceAngleB = revolute.m_referenceAngle;
      m_localAxisD.setZero();

      coordinateB = aB - aD - m_referenceAngleB;
    } else {
      var pB : Vec2 = pool.popVec2();
      var temp : Vec2 = pool.popVec2();
      var prismatic : PrismaticJoint = cast def.joint2;
      m_localAnchorD.setVec(prismatic.m_localAnchorA);
      m_localAnchorB.setVec(prismatic.m_localAnchorB);
      m_referenceAngleB = prismatic.m_referenceAngle;
      m_localAxisD.setVec(prismatic.m_localXAxisA);

      var pD : Vec2 = m_localAnchorD;
      Rot.mulToOutUnsafe(xfB.q, m_localAnchorB, temp);
      temp.addLocalVec(xfB.p).subLocal(xfD.p);
      Rot.mulTransUnsafe2(xfD.q, temp, pB);
      coordinateB = Vec2.dot(pB.subLocal(pD), m_localAxisD);
      pool.pushVec2(2);
    }

    m_ratio = def.ratio;

    m_constant = coordinateA + m_ratio * coordinateB;

    m_impulse = 0.0;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.setVec(m_JvAC).mulLocal(m_impulse);
    argOut.mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    var L : Float = m_impulse * m_JwA;
    return inv_dt * L;
  }

  public function setRatio(argRatio : Float) : Void {
    m_ratio = argRatio;
  }

  public function getRatio() : Float {
    return m_ratio;
  }

  override public function initVelocityConstraints(data : SolverData) : Void {
    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_indexC = m_bodyC.m_islandIndex;
    m_indexD = m_bodyD.m_islandIndex;
    m_lcA.setVec(m_bodyA.m_sweep.localCenter);
    m_lcB.setVec(m_bodyB.m_sweep.localCenter);
    m_lcC.setVec(m_bodyC.m_sweep.localCenter);
    m_lcD.setVec(m_bodyD.m_sweep.localCenter);
    m_mA = m_bodyA.m_invMass;
    m_mB = m_bodyB.m_invMass;
    m_mC = m_bodyC.m_invMass;
    m_mD = m_bodyD.m_invMass;
    m_iA = m_bodyA.m_invI;
    m_iB = m_bodyB.m_invI;
    m_iC = m_bodyC.m_invI;
    m_iD = m_bodyD.m_invI;

    // Vec2 cA = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;

    // Vec2 cB = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    // Vec2 cC = data.positions[m_indexC].c;
    var aC : Float = data.positions[m_indexC].a;
    var vC : Vec2 = data.velocities[m_indexC].v;
    var wC : Float = data.velocities[m_indexC].w;

    // Vec2 cD = data.positions[m_indexD].c;
    var aD : Float = data.positions[m_indexD].a;
    var vD : Vec2 = data.velocities[m_indexD].v;
    var wD : Float = data.velocities[m_indexD].w;

    var qA : Rot = pool.popRot(), qB = pool.popRot(), qC = pool.popRot(), qD = pool.popRot();
    qA.set(aA);
    qB.set(aB);
    qC.set(aC);
    qD.set(aD);

    m_mass = 0.0;

    var temp : Vec2 = pool.popVec2();

    if (m_typeA == JointType.REVOLUTE) {
      m_JvAC.setZero();
      m_JwA = 1.0;
      m_JwC = 1.0;
      m_mass += m_iA + m_iC;
    } else {
      var rC : Vec2 = pool.popVec2();
      var rA : Vec2 = pool.popVec2();
      Rot.mulToOutUnsafe(qC, m_localAxisC, m_JvAC);
      Rot.mulToOutUnsafe(qC, temp.setVec(m_localAnchorC).subLocal(m_lcC), rC);
      Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_lcA), rA);
      m_JwC = Vec2.crossVec(rC, m_JvAC);
      m_JwA = Vec2.crossVec(rA, m_JvAC);
      m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
      pool.pushVec2(2);
    }

    if (m_typeB == JointType.REVOLUTE) {
      m_JvBD.setZero();
      m_JwB = m_ratio;
      m_JwD = m_ratio;
      m_mass += m_ratio * m_ratio * (m_iB + m_iD);
    } else {
      var u : Vec2 = pool.popVec2();
      var rD : Vec2 = pool.popVec2();
      var rB : Vec2 = pool.popVec2();
      Rot.mulToOutUnsafe(qD, m_localAxisD, u);
      Rot.mulToOutUnsafe(qD, temp.setVec(m_localAnchorD).subLocal(m_lcD), rD);
      Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_lcB), rB);
      m_JvBD.setVec(u).mulLocal(m_ratio);
      m_JwD = m_ratio * Vec2.crossVec(rD, u);
      m_JwB = m_ratio * Vec2.crossVec(rB, u);
      m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
      pool.pushVec2(3);
    }

    // Compute effective mass.
    m_mass = m_mass > 0.0 ? 1.0 / m_mass : 0.0;

    if (data.step.warmStarting) {
      vA.x += (m_mA * m_impulse) * m_JvAC.x;
      vA.y += (m_mA * m_impulse) * m_JvAC.y;
      wA += m_iA * m_impulse * m_JwA;

      vB.x += (m_mB * m_impulse) * m_JvBD.x;
      vB.y += (m_mB * m_impulse) * m_JvBD.y;
      wB += m_iB * m_impulse * m_JwB;

      vC.x -= (m_mC * m_impulse) * m_JvAC.x;
      vC.y -= (m_mC * m_impulse) * m_JvAC.y;
      wC -= m_iC * m_impulse * m_JwC;

      vD.x -= (m_mD * m_impulse) * m_JvBD.x;
      vD.y -= (m_mD * m_impulse) * m_JvBD.y;
      wD -= m_iD * m_impulse * m_JwD;
    } else {
      m_impulse = 0.0;
    }
    pool.pushVec2(1);
    pool.pushRot(4);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
    // data.velocities[m_indexC].v = vC;
    data.velocities[m_indexC].w = wC;
    // data.velocities[m_indexD].v = vD;
    data.velocities[m_indexD].w = wD;
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;
    var vC : Vec2 = data.velocities[m_indexC].v;
    var wC : Float = data.velocities[m_indexC].w;
    var vD : Vec2 = data.velocities[m_indexD].v;
    var wD : Float = data.velocities[m_indexD].w;

    var temp1 : Vec2 = pool.popVec2();
    var temp2 : Vec2 = pool.popVec2();
    var Cdot : Float =
        Vec2.dot(m_JvAC, temp1.setVec(vA).subLocal(vC)) + Vec2.dot(m_JvBD, temp2.setVec(vB).subLocal(vD));
    Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);
    pool.pushVec2(2);

    var impulse : Float = -m_mass * Cdot;
    m_impulse += impulse;

    vA.x += (m_mA * impulse) * m_JvAC.x;
    vA.y += (m_mA * impulse) * m_JvAC.y;
    wA += m_iA * impulse * m_JwA;

    vB.x += (m_mB * impulse) * m_JvBD.x;
    vB.y += (m_mB * impulse) * m_JvBD.y;
    wB += m_iB * impulse * m_JwB;

    vC.x -= (m_mC * impulse) * m_JvAC.x;
    vC.y -= (m_mC * impulse) * m_JvAC.y;
    wC -= m_iC * impulse * m_JwC;

    vD.x -= (m_mD * impulse) * m_JvBD.x;
    vD.y -= (m_mD * impulse) * m_JvBD.y;
    wD -= m_iD * impulse * m_JwD;


    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
    // data.velocities[m_indexC].v = vC;
    data.velocities[m_indexC].w = wC;
    // data.velocities[m_indexD].v = vD;
    data.velocities[m_indexD].w = wD;
  }

  public function getJoint1() : Joint {
    return m_joint1;
  }

  public function getJoint2() : Joint {
    return m_joint2;
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    var cA : Vec2 = data.positions[m_indexA].c;
    var aA : Float = data.positions[m_indexA].a;
    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;
    var cC : Vec2 = data.positions[m_indexC].c;
    var aC : Float = data.positions[m_indexC].a;
    var cD : Vec2 = data.positions[m_indexD].c;
    var aD : Float = data.positions[m_indexD].a;

    var qA : Rot = pool.popRot(), qB = pool.popRot(), qC = pool.popRot(), qD = pool.popRot();
    qA.set(aA);
    qB.set(aB);
    qC.set(aC);
    qD.set(aD);

    var linearError : Float = 0.0;

    var coordinateA : Float, coordinateB : Float;

    var temp : Vec2 = pool.popVec2();
    var JvAC : Vec2 = pool.popVec2();
    var JvBD : Vec2 = pool.popVec2();
    var JwA : Float, JwB : Float, JwC : Float, JwD : Float;
    var mass : Float = 0.0;

    if (m_typeA == JointType.REVOLUTE) {
      JvAC.setZero();
      JwA = 1.0;
      JwC = 1.0;
      mass += m_iA + m_iC;

      coordinateA = aA - aC - m_referenceAngleA;
    } else {
      var rC : Vec2 = pool.popVec2();
      var rA : Vec2 = pool.popVec2();
      var pC : Vec2 = pool.popVec2();
      var pA : Vec2 = pool.popVec2();
      Rot.mulToOutUnsafe(qC, m_localAxisC, JvAC);
      Rot.mulToOutUnsafe(qC, temp.setVec(m_localAnchorC).subLocal(m_lcC), rC);
      Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_lcA), rA);
      JwC = Vec2.crossVec(rC, JvAC);
      JwA = Vec2.crossVec(rA, JvAC);
      mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

      pC.setVec(m_localAnchorC).subLocal(m_lcC);
      Rot.mulTransUnsafe2(qC, temp.setVec(rA).addLocalVec(cA).subLocal(cC), pA);
      coordinateA = Vec2.dot(pA.subLocal(pC), m_localAxisC);
      pool.pushVec2(4);
    }

    if (m_typeB == JointType.REVOLUTE) {
      JvBD.setZero();
      JwB = m_ratio;
      JwD = m_ratio;
      mass += m_ratio * m_ratio * (m_iB + m_iD);

      coordinateB = aB - aD - m_referenceAngleB;
    } else {
      var u : Vec2 = pool.popVec2();
      var rD : Vec2 = pool.popVec2();
      var rB : Vec2 = pool.popVec2();
      var pD : Vec2 = pool.popVec2();
      var pB : Vec2 = pool.popVec2();
      Rot.mulToOutUnsafe(qD, m_localAxisD, u);
      Rot.mulToOutUnsafe(qD, temp.setVec(m_localAnchorD).subLocal(m_lcD), rD);
      Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_lcB), rB);
      JvBD.setVec(u).mulLocal(m_ratio);
      JwD = Vec2.crossVec(rD, u);
      JwB = Vec2.crossVec(rB, u);
      mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

      pD.setVec(m_localAnchorD).subLocal(m_lcD);
      Rot.mulTransUnsafe2(qD, temp.setVec(rB).addLocalVec(cB).subLocal(cD), pB);
      coordinateB = Vec2.dot(pB.subLocal(pD), m_localAxisD);
      pool.pushVec2(5);
    }

    var C : Float = (coordinateA + m_ratio * coordinateB) - m_constant;

    var impulse : Float = 0.0;
    if (mass > 0.0) {
      impulse = -C / mass;
    }
    pool.pushVec2(3);
    pool.pushRot(4);

    cA.x += (m_mA * impulse) * JvAC.x;
    cA.y += (m_mA * impulse) * JvAC.y;
    aA += m_iA * impulse * JwA;

    cB.x += (m_mB * impulse) * JvBD.x;
    cB.y += (m_mB * impulse) * JvBD.y;
    aB += m_iB * impulse * JwB;

    cC.x -= (m_mC * impulse) * JvAC.x;
    cC.y -= (m_mC * impulse) * JvAC.y;
    aC -= m_iC * impulse * JwC;

    cD.x -= (m_mD * impulse) * JvBD.x;
    cD.y -= (m_mD * impulse) * JvBD.y;
    aD -= m_iD * impulse * JwD;

    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;
    // data.positions[m_indexC].c = cC;
    data.positions[m_indexC].a = aC;
    // data.positions[m_indexD].c = cD;
    data.positions[m_indexD].a = aD;

    // TODO_ERIN not implemented
    return linearError < Settings.linearSlop;
  }
}

