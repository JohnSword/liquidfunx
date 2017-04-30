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
 * Created at 7:27:32 AM Jan 20, 2011
 */
package box2d.dynamics.joints;

import box2d.common.Mat22;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

/**
 * @author Daniel Murphy
 */
 class FrictionJoint extends Joint {

  private var m_localAnchorA : Vec2;
  private var m_localAnchorB : Vec2;

  // Solver shared
  private var m_linearImpulse : Vec2;
  private var m_angularImpulse : Float;
  private var m_maxForce : Float;
  private var m_maxTorque : Float;

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
  private var m_linearMass : Mat22 = new Mat22();
  private var m_angularMass : Float;

  public function new(argWorldPool : IWorldPool, def : FrictionJointDef) {
    super(argWorldPool, def);
    m_localAnchorA = new Vec2().setVec(def.localAnchorA);
    m_localAnchorB = new Vec2().setVec(def.localAnchorB);

    m_linearImpulse = new Vec2();
    m_angularImpulse = 0.0;

    m_maxForce = def.maxForce;
    m_maxTorque = def.maxTorque;
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
    argOut.setVec(m_linearImpulse).mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return inv_dt * m_angularImpulse;
  }

  public function setMaxForce(force : Float) : Void {
    m_maxForce = force;
  }

  public function getMaxForce() : Float {
    return m_maxForce;
  }

  public function setMaxTorque(torque : Float) : Void {
    m_maxTorque = torque;
  }

  public function getMaxTorque() : Float {
    return m_maxTorque;
  }

  /**
   * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
   */
  override public function initVelocityConstraints(data : SolverData) : Void {
    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterA.setVec(m_bodyA.m_sweep.localCenter);
    m_localCenterB.setVec(m_bodyB.m_sweep.localCenter);
    m_invMassA = m_bodyA.m_invMass;
    m_invMassB = m_bodyB.m_invMass;
    m_invIA = m_bodyA.m_invI;
    m_invIB = m_bodyB.m_invI;

    var aA : Float = data.positions[m_indexA].a;
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;

    var aB : Float = data.positions[m_indexB].a;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;


    var temp : Vec2 = pool.popVec2();
    var qA : Rot = pool.popRot();
    var qB : Rot = pool.popRot();

    qA.set(aA);
    qB.set(aB);

    // Compute the effective mass matrix.
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

    var K : Mat22 = pool.popMat22();
    K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
    K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

    K.invertToOut(m_linearMass);

    m_angularMass = iA + iB;
    if (m_angularMass > 0.0) {
      m_angularMass = 1.0 / m_angularMass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      m_linearImpulse.mulLocal(data.step.dtRatio);
      m_angularImpulse *= data.step.dtRatio;

      var P : Vec2 = pool.popVec2();
      P.setVec(m_linearImpulse);

      temp.setVec(P).mulLocal(mA);
      vA.subLocal(temp);
      wA -= iA * (Vec2.crossVec(m_rA, P) + m_angularImpulse);

      temp.setVec(P).mulLocal(mB);
      vB.addLocalVec(temp);
      wB += iB * (Vec2.crossVec(m_rB, P) + m_angularImpulse);

      pool.pushVec2(1);
    } else {
      m_linearImpulse.setZero();
      m_angularImpulse = 0.0;
    }
//    data.velocities[m_indexA].v.set(vA);
    if( data.velocities[m_indexA].w != wA) {
      // assert(data.velocities[m_indexA].w != wA);
    }
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(1);
    pool.pushMat22(1);
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var h : Float = data.step.dt;

    // Solve angular friction
    {
      var Cdot : Float = wB - wA;
      var impulse : Float = -m_angularMass * Cdot;

      var oldImpulse : Float = m_angularImpulse;
      var maxImpulse : Float = h * m_maxTorque;
      m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve linear friction
    {
      var Cdot : Vec2 = pool.popVec2();
      var temp : Vec2 = pool.popVec2();

      Vec2.crossToOutUnsafeFVV(wA, m_rA, temp);
      Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot);
      Cdot.addLocalVec(vB).subLocal(vA).subLocal(temp);

      var impulse : Vec2 = pool.popVec2();
      Mat22.mulToOutUnsafeMVV(m_linearMass, Cdot, impulse);
      impulse.negateLocal();


      var oldImpulse : Vec2 = pool.popVec2();
      oldImpulse.setVec(m_linearImpulse);
      m_linearImpulse.addLocalVec(impulse);

      var maxImpulse : Float = h * m_maxForce;

      if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
        m_linearImpulse.normalize();
        m_linearImpulse.mulLocal(maxImpulse);
      }

      impulse.setVec(m_linearImpulse).subLocal(oldImpulse);

      temp.setVec(impulse).mulLocal(mA);
      vA.subLocal(temp);
      wA -= iA * Vec2.crossVec(m_rA, impulse);

      temp.setVec(impulse).mulLocal(mB);
      vB.addLocalVec(temp);
      wB += iB * Vec2.crossVec(m_rB, impulse);
      
    }

//    data.velocities[m_indexA].v.set(vA);
    if( data.velocities[m_indexA].w != wA) {
      // assert(data.velocities[m_indexA].w != wA);
    }
    data.velocities[m_indexA].w = wA;
   
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(4);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    return true;
  }
}

