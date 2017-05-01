package box2d.dynamics.joints;

import box2d.common.Mat22;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

//Point-to-point constraint
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Angle constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A motor joint is used to control the relative motion between two bodies. A typical usage is to
 * control the movement of a dynamic body with respect to the ground.
 * 
 * @author dmurph
 */
 class MotorJoint extends Joint {

  // Solver shared
  private var m_linearOffset : Vec2 = new Vec2();
  private var m_angularOffset : Float;
  private var m_linearImpulse : Vec2 = new Vec2();
  private var m_angularImpulse : Float;
  private var m_maxForce : Float;
  private var m_maxTorque : Float;
  private var m_correctionFactor : Float;

  // Solver temp
  private var m_indexA : Int;
  private var m_indexB : Int;
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_linearError : Vec2 = new Vec2();
  private var m_angularError : Float;
  private var m_invMassA : Float;
  private var m_invMassB : Float;
  private var m_invIA : Float;
  private var m_invIB : Float;
  private var m_linearMass : Mat22 = new Mat22();
  private var m_angularMass : Float;

  public function new(pool : IWorldPool, def : MotorJointDef) {
    super(pool, def);
    m_linearOffset.setVec(def.linearOffset);
    m_angularOffset = def.angularOffset;

    m_angularImpulse = 0.0;

    m_maxForce = def.maxForce;
    m_maxTorque = def.maxTorque;
    m_correctionFactor = def.correctionFactor;
  }

  override public function getAnchorA(out : Vec2) : Void {
    out.setVec(m_bodyA.getPosition());
  }

  override public function getAnchorB(out : Vec2) : Void {
    out.setVec(m_bodyB.getPosition());
  }

  override public function getReactionForce(inv_dt : Float, out : Vec2) : Void {
    out.setVec(m_linearImpulse).mulLocal(inv_dt);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return m_angularImpulse * inv_dt;
  }

  public function getCorrectionFactor() : Float {
    return m_correctionFactor;
  }

  public function setCorrectionFactor(correctionFactor : Float) : Void {
    this.m_correctionFactor = correctionFactor;
  }

  /**
   * Set the target linear offset, in frame A, in meters.
   */
  public function setLinearOffset(linearOffset : Vec2) : Void {
    if (linearOffset.x != m_linearOffset.x || linearOffset.y != m_linearOffset.y) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_linearOffset.setVec(linearOffset);
    }
  }

  /**
   * Get the target linear offset, in frame A, in meters.
   */
  public function getLinearOffset(out : Vec2) : Void {
    out.setVec(m_linearOffset);
  }

  /**
   * Get the target linear offset, in frame A, in meters. Do not modify.
   */
  public function getLinearOffsetVec() : Vec2 {
    return m_linearOffset;
  }

  /**
   * Set the target angular offset, in radians.
   * 
   * @param angularOffset
   */
  public function setAngularOffset(angularOffset : Float) : Void {
    if (angularOffset != m_angularOffset) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_angularOffset = angularOffset;
    }
  }

  public function getAngularOffset() : Float {
    return m_angularOffset;
  }

  /**
   * Set the maximum friction force in N.
   * 
   * @param force
   */
  public function setMaxForce(force : Float) : Void {
    m_maxForce = force;
  }

  /**
   * Get the maximum friction force in N.
   */
  public function getMaxForce() : Float {
    return m_maxForce;
  }

  /**
   * Set the maximum friction torque in N*m.
   */
  public function setMaxTorque(torque : Float) : Void {
    m_maxTorque = torque;
  }

  /**
   * Get the maximum friction torque in N*m.
   */
  public function getMaxTorque() : Float {
    return m_maxTorque;
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
    var K : Mat22 = pool.popMat22();

    qA.set(aA);
    qB.set(aB);

    // Compute the effective mass matrix.
    // m_rA = b2Mul(qA, -m_localCenterA);
    // m_rB = b2Mul(qB, -m_localCenterB);
    m_rA.x = qA.c * -m_localCenterA.x - qA.s * -m_localCenterA.y;
    m_rA.y = qA.s * -m_localCenterA.x + qA.c * -m_localCenterA.y;
    m_rB.x = qB.c * -m_localCenterB.x - qB.s * -m_localCenterB.y;
    m_rB.y = qB.s * -m_localCenterB.x + qB.c * -m_localCenterB.y;

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
    K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

    K.invertToOut(m_linearMass);

    m_angularMass = iA + iB;
    if (m_angularMass > 0.0) {
      m_angularMass = 1.0 / m_angularMass;
    }

    // m_linearError = cB + m_rB - cA - m_rA - b2Mul(qA, m_linearOffset);
    Rot.mulToOutUnsafe(qA, m_linearOffset, temp);
    m_linearError.x = cB.x + m_rB.x - cA.x - m_rA.x - temp.x;
    m_linearError.y = cB.y + m_rB.y - cA.y - m_rA.y - temp.y;
    m_angularError = aB - aA - m_angularOffset;

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      m_linearImpulse.x *= data.step.dtRatio;
      m_linearImpulse.y *= data.step.dtRatio;
      m_angularImpulse *= data.step.dtRatio;

      var P : Vec2 = m_linearImpulse;
      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (m_rA.x * P.y - m_rA.y * P.x + m_angularImpulse);
      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (m_rB.x * P.y - m_rB.y * P.x + m_angularImpulse);
    } else {
      m_linearImpulse.setZero();
      m_angularImpulse = 0.0;
    }

    pool.pushVec2(1);
    pool.pushMat22(1);
    pool.pushRot(2);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  override public function solveVelocityConstraints(data : SolverData) : Void {
    var vA : Vec2 = data.velocities[m_indexA].v;
    var wA : Float = data.velocities[m_indexA].w;
    var vB : Vec2 = data.velocities[m_indexB].v;
    var wB : Float = data.velocities[m_indexB].w;

    var mA : Float = m_invMassA, mB = m_invMassB;
    var iA : Float = m_invIA, iB = m_invIB;

    var h : Float = data.step.dt;
    var inv_h : Float = data.step.inv_dt;

    var temp : Vec2 = pool.popVec2();

    // Solve angular friction
    {
      var Cdot : Float = wB - wA + inv_h * m_correctionFactor * m_angularError;
      var impulse : Float = -m_angularMass * Cdot;

      var oldImpulse : Float = m_angularImpulse;
      var maxImpulse : Float = h * m_maxTorque;
      m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    var Cdot : Vec2 = pool.popVec2();

    // Solve linear friction
    {
      // Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA) + inv_h * m_correctionFactor *
      // m_linearError;
      Cdot.x =
          vB.x + -wB * m_rB.y - vA.x - -wA * m_rA.y + inv_h * m_correctionFactor * m_linearError.x;
      Cdot.y =
          vB.y + wB * m_rB.x - vA.y - wA * m_rA.x + inv_h * m_correctionFactor * m_linearError.y;

      var impulse : Vec2 = temp;
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

      impulse.x = m_linearImpulse.x - oldImpulse.x;
      impulse.y = m_linearImpulse.y - oldImpulse.y;

      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * (m_rA.x * impulse.y - m_rA.y * impulse.x);

      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * (m_rB.x * impulse.y - m_rB.y * impulse.x);
    }

    pool.pushVec2(3);

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;
  }

  override public function solvePositionConstraints(data : SolverData) : Bool {
    return true;
  }
}

