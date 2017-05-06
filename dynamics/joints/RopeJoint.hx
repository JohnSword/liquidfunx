package box2d.dynamics.joints;

import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.dynamics.SolverData;
import box2d.pooling.IWorldPool;

/**
 * A rope joint enforces a maximum distance between two points on two bodies. It has no other
 * effect. Warning: if you attempt to change the maximum length during the simulation you will get
 * some non-physical behavior. A model that would allow you to dynamically modify the length would
 * have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
 * dynamically control length.
 * 
 * @author Daniel Murphy
 */
 class RopeJoint extends Joint {
  // Solver shared
  private var m_localAnchorA : Vec2 = new Vec2();
  private var m_localAnchorB : Vec2 = new Vec2();
  private var m_maxLength : Float = 0;
  private var m_length : Float = 0;
  private var m_impulse : Float = 0;

  // Solver temp
  private var m_indexA : Int = 0;
  private var m_indexB : Int = 0;
  private var m_u : Vec2 = new Vec2();
  private var m_rA : Vec2 = new Vec2();
  private var m_rB : Vec2 = new Vec2();
  private var m_localCenterA : Vec2 = new Vec2();
  private var m_localCenterB : Vec2 = new Vec2();
  private var m_invMassA : Float = 0;
  private var m_invMassB : Float = 0;
  private var m_invIA : Float = 0;
  private var m_invIB : Float = 0;
  private var m_mass : Float = 0;
  private var m_state : LimitState;

  public function new(worldPool : IWorldPool, def : RopeJointDef) {
    super(worldPool, def);
    m_localAnchorA.setVec(def.localAnchorA);
    m_localAnchorB.setVec(def.localAnchorB);

    m_maxLength = def.maxLength;

    m_mass = 0.0;
    m_impulse = 0.0;
    m_state = LimitState.INACTIVE;
    m_length = 0.0;
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

    m_u.setVec(cB).addLocalVec(m_rB).subLocal(cA).subLocal(m_rA);

    m_length = m_u.length();

    var C : Float = m_length - m_maxLength;
    if (C > 0.0) {
      m_state = LimitState.AT_UPPER;
    } else {
      m_state = LimitState.INACTIVE;
    }

    if (m_length > Settings.linearSlop) {
      m_u.mulLocal(1.0 / m_length);
    } else {
      m_u.setZero();
      m_mass = 0.0;
      m_impulse = 0.0;
      return;
    }

    // Compute effective mass.
    var crA : Float = Vec2.crossVec(m_rA, m_u);
    var crB : Float = Vec2.crossVec(m_rB, m_u);
    var invMass : Float = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

    m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      m_impulse *= data.step.dtRatio;

      var Px : Float = m_impulse * m_u.x;
      var Py : Float = m_impulse * m_u.y;
      vA.x -= m_invMassA * Px;
      vA.y -= m_invMassA * Py;
      wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);

      vB.x += m_invMassB * Px;
      vB.y += m_invMassB * Py;
      wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);
    } else {
      m_impulse = 0.0;
    }

    pool.pushRot(2);
    pool.pushVec2(1);

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

    // Cdot = dot(u, v + cross(w, r))
    var vpA : Vec2 = pool.popVec2();
    var vpB : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();

    Vec2.crossToOutUnsafeFVV(wA, m_rA, vpA);
    vpA.addLocalVec(vA);
    Vec2.crossToOutUnsafeFVV(wB, m_rB, vpB);
    vpB.addLocalVec(vB);

    var C : Float = m_length - m_maxLength;
    var Cdot : Float = Vec2.dot(m_u, temp.setVec(vpB).subLocal(vpA));

    // Predictive constraint.
    if (C < 0.0) {
      Cdot += data.step.inv_dt * C;
    }

    var impulse : Float = -m_mass * Cdot;
    var oldImpulse : Float = m_impulse;
    m_impulse = MathUtils.min(0.0, m_impulse + impulse);
    impulse = m_impulse - oldImpulse;

    var Px : Float = impulse * m_u.x;
    var Py : Float = impulse * m_u.y;
    vA.x -= m_invMassA * Px;
    vA.y -= m_invMassA * Py;
    wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);
    vB.x += m_invMassB * Px;
    vB.y += m_invMassB * Py;
    wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);

    pool.pushVec2(3);

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
    var u : Vec2 = pool.popVec2();
    var rA : Vec2 = pool.popVec2();
    var rB : Vec2 = pool.popVec2();
    var temp : Vec2 = pool.popVec2();

    qA.set(aA);
    qB.set(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, temp.setVec(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.setVec(m_localAnchorB).subLocal(m_localCenterB), rB);
    u.setVec(cB).addLocalVec(rB).subLocal(cA).subLocal(rA);

    var length : Float = u.normalize();
    var C : Float = length - m_maxLength;

    C = MathUtils.clamp(C, 0.0, Settings.maxLinearCorrection);

    var impulse : Float = -m_mass * C;
    var Px : Float = impulse * u.x;
    var Py : Float = impulse * u.y;

    cA.x -= m_invMassA * Px;
    cA.y -= m_invMassA * Py;
    aA -= m_invIA * (rA.x * Py - rA.y * Px);
    cB.x += m_invMassB * Px;
    cB.y += m_invMassB * Py;
    aB += m_invIB * (rB.x * Py - rB.y * Px);

    pool.pushRot(2);
    pool.pushVec2(4);

    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return length - m_maxLength < Settings.linearSlop;
  }

  override public function getAnchorA(argOut : Vec2) : Void {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  override public function getAnchorB(argOut : Vec2) : Void {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {
    argOut.setVec(m_u).mulLocal(inv_dt).mulLocal(m_impulse);
  }

  override public function getReactionTorque(inv_dt : Float) : Float {
    return 0;
  }

  public function getLocalAnchorA() : Vec2 {
    return m_localAnchorA;
  }

  public function getLocalAnchorB() : Vec2 {
    return m_localAnchorB;
  }

  public function getMaxLength() : Float {
    return m_maxLength;
  }

  public function setMaxLength(maxLength : Float) : Void {
    this.m_maxLength = maxLength;
  }

  public function getLimitState() : LimitState {
    return m_state;
  }

}

