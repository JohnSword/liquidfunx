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
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

/**
 * A mouse joint is used to make a point on a body track a specified world point. This a soft
 * constraint with a maximum force. This allows the constraint to stretch and without applying huge
 * forces. NOTE: this joint is not documented in the manual because it was developed to be used in
 * the testbed. If you want to learn how to use the mouse joint, look at the testbed.
 * 
 * @author Daniel
 */
 class MouseJoint extends Joint {

  private var m_localAnchorB : Vec2 = new Vec2();
  private var m_targetA : Vec2 = new Vec2();
  private var m_frequencyHz : Float;
  private var m_dampingRatio : Float;
  private var m_beta : Float;

  // Solver shared
  private var m_impulse : Vec2 = new Vec2();
  private var m_maxForce : Float;
  private var m_gamma : Float;

  // Solver temp
  private var m_indexB : Int;
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassB : Float;
  private var m_invIB : Float;
  private var m_mass : Mat22 = new Mat22();
  private var m_C : Vec2 = new Vec2();

  public function new(argWorld : IWorldPool, def : MouseJointDef) {
    super(argWorld, def);

    m_targetA.setVec(def.target);
    Transform.mulTransToOutUnsafe(m_bodyB.getTransform(), m_targetA, m_localAnchorB);

    m_maxForce = def.maxForce;
    m_impulse.setZero();

    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;

    m_beta = 0;
    m_gamma = 0;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    argOut.setVec(m_targetA);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(invDt : Float, argOut : Vec2) : Void {
    argOut.setVec(m_impulse).mulLocal(invDt);
  }

  override public function getReactionTorque(invDt : Float) : Float {
    return invDt * 0.0;
  }


  public function setTarget(target : Vec2) : Void {
    if (m_bodyB.isAwake() == false) {
      m_bodyB.setAwake(true);
    }
    m_targetA.setVec(target);
  }

  public function getTarget() : Vec2 {
    return m_targetA;
  }

  // / set/get the maximum force in Newtons.
  public function setMaxForce(force : Float) : Void {
    m_maxForce = force;
  }

  public function getMaxForce() : Float {
    return m_maxForce;
  }

  // / set/get the frequency in Hertz.
  public function setFrequency(hz : Float) : Void {
    m_frequencyHz = hz;
  }

  public function getFrequency() : Float {
    return m_frequencyHz;
  }

  // / set/get the damping ratio (dimensionless).
  public function setDampingRatio(ratio : Float) : Void {
    m_dampingRatio = ratio;
  }

  public function getDampingRatio() : Float {
    return m_dampingRatio;
  }

  override public function initVelocityConstraints(data : SolverData) : Void {
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterB.setVec(m_bodyB.m_sweep.localCenter);
    m_invMassB = m_bodyB.m_invMass;
    m_invIB = m_bodyB.m_invI;

    var cB : Vec2 = data.positions[m_indexB].c;
    var aB : Float = data.positions[m_indexB].a;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var qB : Rot = pool.popRot();

    qB.set(aB);

    var mass : Float = m_bodyB.getMass();

    // Frequency
    var omega : Float = 2.0 * MathUtils.PI * m_frequencyHz;

    // Damping coefficient
    var d : Float = 2.0 * mass * m_dampingRatio * omega;

    // Spring stiffness
    var k : Float = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    var h : Float = data.step.dt;
    m_gamma = h * (d + h * k);
    if (m_gamma != 0.0) {
      m_gamma = 1.0 / m_gamma;
    }
    m_beta = h * k * m_gamma;

    var temp : Vec2 = pool.popVec2();

    // Compute the effective mass matrix.
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), m_rB);

    // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
    var K : Mat22 = pool.popMat22();
    K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
    K.ex.y = -m_invIB * m_rB.x * m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

    K.invertToOut(m_mass);

    m_C.setVec(cB).addLocalVec(m_rB).subLocal(m_targetA);
    m_C.mulLocal(m_beta);

    // Cheat with some damping
    wB *= 0.98;

    if (data.step.warmStarting) {
      m_impulse.mulLocal(data.step.dtRatio);
      vB.x += m_invMassB * m_impulse.x;
      vB.y += m_invMassB * m_impulse.y;
      wB += m_invIB * Vec2.crossVec(m_rB, m_impulse);
    } else {
      m_impulse.setZero();
    }

//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushMat22(1);
    pool.pushRot(1);
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    return true;
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {

    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    // Cdot = v + cross(w, r)
    var Cdot : Vec2 = pool.popVec2();
    Vec2.crossToOutUnsafeFVV(wB, m_rB, Cdot);
    Cdot.addLocalVec(vB);

    var impulse : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();

    temp.setVec(m_impulse).mulLocal(m_gamma).addLocalVec(m_C).addLocalVec(Cdot).negateLocal();
    Mat22.mulToOutUnsafeMVV(m_mass, temp, impulse);

    var oldImpulse : Vec2 = temp;
    oldImpulse.setVec(m_impulse);
    m_impulse.addLocalVec(impulse);
    var maxImpulse : Float = data.step.dt * m_maxForce;
    if (m_impulse.lengthSquared() > maxImpulse * maxImpulse) {
      m_impulse.mulLocal(maxImpulse / m_impulse.length());
    }
    impulse.setVec(m_impulse).subLocal(oldImpulse);

    vB.x += m_invMassB * impulse.x;
    vB.y += m_invMassB * impulse.y;
    wB += m_invIB * Vec2.crossVec(m_rB, impulse);

//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;
    
    pool.pushVec2(3);
  }

}

