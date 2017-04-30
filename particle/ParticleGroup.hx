package box2d.particle;

import box2d.common.Transform;
import box2d.common.Vec2;

class ParticleGroup {

  public var m_system : ParticleSystem;
  public var m_firstIndex : Int;
  public var m_lastIndex : Int;
  public var m_groupFlags : Int;
  public var m_strength : Float;
  public var m_prev : ParticleGroup;
  public var m_next : ParticleGroup;

  public var m_timestamp : Int;
  public var m_mass : Float;
  public var m_inertia : Float;
  public var m_center : Vec2 = new Vec2();
  public var m_linearVelocity : Vec2 = new Vec2();
  public var m_angularVelocity : Float;
  public var m_transform : Transform = new Transform();

  public var m_destroyAutomatically : Bool;
  public var m_toBeDestroyed : Bool;
  public var m_toBeSplit : Bool;

  public var m_userData : Dynamic;

  public function new() {
    m_firstIndex = 0;
    m_lastIndex = 0;
    m_groupFlags = 0;
    m_strength = 1.0;

    m_timestamp = -1;
    m_mass = 0;
    m_inertia = 0;
    m_angularVelocity = 0;
    m_transform.setIdentity();

    m_destroyAutomatically = true;
    m_toBeDestroyed = false;
    m_toBeSplit = false;
  }

  public function getNext() : ParticleGroup {
    return m_next;
  }

  public function getParticleCount() : Int {
    return m_lastIndex - m_firstIndex;
  }

  public function getBufferIndex() : Int {
    return m_firstIndex;
  }

  public function getGroupFlags() : Int {
    return m_groupFlags;
  }

  public function setGroupFlags(flags : Int) : Void {
    m_groupFlags = flags;
  }

  public function getMass() : Float {
    updateStatistics();
    return m_mass;
  }

  public function getInertia() : Float {
    updateStatistics();
    return m_inertia;
  }

  public function getCenter() : Vec2 {
    updateStatistics();
    return m_center;
  }

  public function getLinearVelocity() : Vec2 {
    updateStatistics();
    return m_linearVelocity;
  }

  public function getAngularVelocity() : Float {
    updateStatistics();
    return m_angularVelocity;
  }

  public function getTransform() : Transform {
    return m_transform;
  }

  public function getPosition() : Vec2 {
    return m_transform.p;
  }

  public function getAngle() : Float {
    return m_transform.q.getAngle();
  }

  public function getUserData() : Dynamic {
    return m_userData;
  }

  public function setUserData(data : Dynamic) : Void {
    m_userData = data;
  }
  
  

  public function updateStatistics() : Void {
    if (m_timestamp != m_system.m_timestamp) {
      var m : Float = m_system.getParticleMass();
      m_mass = 0;
      m_center.setZero();
      m_linearVelocity.setZero();
      for(i in m_firstIndex ... m_lastIndex) {
        m_mass += m;
        var pos : Vec2 = m_system.m_positionBuffer.data[i];
        m_center.x += m * pos.x;
        m_center.y += m * pos.y;
        var vel : Vec2 = m_system.m_velocityBuffer.data[i];
        m_linearVelocity.x += m * vel.x;
        m_linearVelocity.y += m * vel.y;
      }
      if (m_mass > 0) {
        m_center.x *= 1 / m_mass;
        m_center.y *= 1 / m_mass;
        m_linearVelocity.x *= 1 / m_mass;
        m_linearVelocity.y *= 1 / m_mass;
      }
      m_inertia = 0;
      m_angularVelocity = 0;
      for(i in m_firstIndex ... m_lastIndex) {
        var pos : Vec2 = m_system.m_positionBuffer.data[i];
        var vel : Vec2 = m_system.m_velocityBuffer.data[i];
        var px : Float = pos.x - m_center.x;
        var py : Float = pos.y - m_center.y;
        var vx : Float = vel.x - m_linearVelocity.x;
        var vy : Float = vel.y - m_linearVelocity.y;
        m_inertia += m * (px * px + py * py);
        m_angularVelocity += m * (px * vy - py * vx);
      }
      if (m_inertia > 0) {
        m_angularVelocity *= 1 / m_inertia;
      }
      m_timestamp = m_system.m_timestamp;
    }
  }
}

