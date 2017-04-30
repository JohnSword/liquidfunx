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
package box2d.dynamics.contacts;


import box2d.callbacks.ContactListener;
import box2d.collision.ContactID;
import box2d.collision.Manifold;
import box2d.collision.ManifoldPoint;
import box2d.collision.WorldManifold;
import box2d.collision.shapes.Shape;
import box2d.common.MathUtils;
import box2d.common.Transform;
import box2d.dynamics.Body;
import box2d.dynamics.Fixture;
import box2d.pooling.IWorldPool;

/**
 * The class manages contact between two shapes. A contact exists for each overlapping AABB in the
 * broad-phase (except if filtered). Therefore a contact object may exist that has no contact
 * points.
 * 
 * @author daniel
 */
class Contact {

  // Flags stored in m_flags
  // Used when crawling contact graph when forming islands.
  public static var ISLAND_FLAG : Int = 0x0001;
  // Set when the shapes are touching.
  public static var TOUCHING_FLAG : Int = 0x0002;
  // This contact can be disabled (by user)
  public static var ENABLED_FLAG : Int = 0x0004;
  // This contact needs filtering because a fixture filter was changed.
  public static var FILTER_FLAG : Int = 0x0008;
  // This bullet contact had a TOI event
  public static var BULLET_HIT_FLAG : Int = 0x0010;

  public static var TOI_FLAG : Int = 0x0020;

  public var m_flags : Int;

  // World pool and list pointers.
  public var m_prev : Contact;
  public var m_next : Contact;

  // Nodes for connecting bodies.
  public var m_nodeA : ContactEdge = null;
  public var m_nodeB : ContactEdge = null;

  public var m_fixtureA : Fixture;
  public var m_fixtureB : Fixture;

  public var m_indexA : Int;
  public var m_indexB : Int;

  public var m_manifold : Manifold;

  public var m_toiCount : Float;
  public var m_toi : Float;

  public var m_friction : Float;
  public var m_restitution : Float;

  public var m_tangentSpeed : Float;

  public var pool : IWorldPool;

  public function new(argPool : IWorldPool) {
    m_fixtureA = null;
    m_fixtureB = null;
    m_nodeA = new ContactEdge();
    m_nodeB = new ContactEdge();
    m_manifold = new Manifold();
    pool = argPool;
  }

  /** initialization for pooling */
  public function init(fA : Fixture, indexA : Int, fB : Fixture, indexB : Int) : Void {
    m_flags = ENABLED_FLAG;

    m_fixtureA = fA;
    m_fixtureB = fB;

    m_indexA = indexA;
    m_indexB = indexB;

    m_manifold.pointCount = 0;

    m_prev = null;
    m_next = null;

    m_nodeA.contact = null;
    m_nodeA.prev = null;
    m_nodeA.next = null;
    m_nodeA.other = null;

    m_nodeB.contact = null;
    m_nodeB.prev = null;
    m_nodeB.next = null;
    m_nodeB.other = null;

    m_toiCount = 0;
    m_friction = Contact.mixFriction(fA.m_friction, fB.m_friction);
    m_restitution = Contact.mixRestitution(fA.m_restitution, fB.m_restitution);

    m_tangentSpeed = 0;
  }

  /**
   * Get the contact manifold. Do not set the point count to zero. Instead call Disable.
   */
  public function getManifold() : Manifold {
    return m_manifold;
  }

  /**
   * Get the world manifold.
   */
  public function getWorldManifold(worldManifold : WorldManifold) : Void {
    var bodyA : Body = m_fixtureA.getBody();
    var bodyB : Body = m_fixtureB.getBody();
    var shapeA : Shape = m_fixtureA.getShape();
    var shapeB : Shape = m_fixtureB.getShape();

    worldManifold.initialize(m_manifold, bodyA.getTransform(), shapeA.m_radius,
        bodyB.getTransform(), shapeB.m_radius);
  }

  /**
   * Is this contact touching
   * 
   * @return
   */
  public function isTouching() : Bool {
    return (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
  }

  /**
   * Enable/disable this contact. This can be used inside the pre-solve contact listener. The
   * contact is only disabled for the current time step (or sub-step in continuous collisions).
   * 
   * @param flag
   */
  public function setEnabled(flag : Bool) : Void {
    if (flag) {
      m_flags |= ENABLED_FLAG;
    } else {
      m_flags &= ~ENABLED_FLAG;
    }
  }

  /**
   * Has this contact been disabled?
   * 
   * @return
   */
  public function isEnabled() : Bool {
    return (m_flags & ENABLED_FLAG) == ENABLED_FLAG;
  }

  /**
   * Get the next contact in the world's contact list.
   * 
   * @return
   */
  public function getNext() : Contact {
    return m_next;
  }

  /**
   * Get the first fixture in this contact.
   * 
   * @return
   */
  public function getFixtureA() : Fixture {
    return m_fixtureA;
  }

  public function getChildIndexA() : Int {
    return m_indexA;
  }

  /**
   * Get the second fixture in this contact.
   * 
   * @return
   */
  public function getFixtureB() : Fixture {
    return m_fixtureB;
  }

  public function getChildIndexB() : Int {
    return m_indexB;
  }

  public function setFriction(friction : Float) : Void {
    m_friction = friction;
  }

  public function getFriction() : Float {
    return m_friction;
  }

  public function resetFriction() : Void {
    m_friction = Contact.mixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
  }

  public function setRestitution(restitution : Float) : Void {
    m_restitution = restitution;
  }

  public function getRestitution() : Float {
    return m_restitution;
  }

  public function resetRestitution() : Void {
    m_restitution = Contact.mixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
  }

  public function setTangentSpeed(speed : Float) : Void {
    m_tangentSpeed = speed;
  }

  public function getTangentSpeed() : Float {
    return m_tangentSpeed;
  }

  public function evaluate(manifold : Manifold, xfA : Transform, xfB : Transform) : Void{
   
  }

  /**
   * Flag this contact for filtering. Filtering will occur the next time step.
   */
  public function flagForFiltering() : Void {
    m_flags |= FILTER_FLAG;
  }

  // djm pooling
  private var oldManifold : Manifold = new Manifold();

  public function update(listener : ContactListener) : Void {

    oldManifold.set(m_manifold);

    // Re-enable this contact.
    m_flags |= ENABLED_FLAG;

    var touching : Bool = false;
    var wasTouching : Bool =  (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;

    var sensorA : Bool = m_fixtureA.isSensor();
    var sensorB : Bool = m_fixtureB.isSensor();
    var sensor : Bool = sensorA || sensorB;

    var bodyA : Body = m_fixtureA.getBody();
    var bodyB : Body = m_fixtureB.getBody();
    var xfA : Transform = bodyA.getTransform();
    var xfB : Transform = bodyB.getTransform();
    // log.debug("TransformA: "+xfA);
    // log.debug("TransformB: "+xfB);

    if (sensor) {
      var shapeA : Shape = m_fixtureA.getShape();
      var shapeB : Shape = m_fixtureB.getShape();
      touching = pool.getCollision().testOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

      // Sensors don't generate manifolds.
      m_manifold.pointCount = 0;
    } else {
      evaluate(m_manifold, xfA, xfB);
      touching = m_manifold.pointCount > 0;

      // Match old contact ids to new contact ids and copy the
      // stored impulses to warm start the solver.
      var i:Int = 0;
      while (i < m_manifold.pointCount) {
      // for (int i = 0; i < m_manifold.pointCount; ++i) {
        var mp2 : ManifoldPoint = m_manifold.points[i];
        mp2.normalImpulse = 0.0;
        mp2.tangentImpulse = 0.0;
        var id2 : ContactID = mp2.id;

        var j:Int = 0;
        while (j < oldManifold.pointCount) {
        // for (int j = 0; j < oldManifold.pointCount; ++j) {
          var mp1 : ManifoldPoint = oldManifold.points[j];

          if (mp1.id.isEqual(id2)) {
            mp2.normalImpulse = mp1.normalImpulse;
            mp2.tangentImpulse = mp1.tangentImpulse;
            break;
          }
          ++j;
        }
        ++i;
      }

      if (touching != wasTouching) {
        bodyA.setAwake(true);
        bodyB.setAwake(true);
      }
    }

    if (touching) {
      m_flags |= TOUCHING_FLAG;
    } else {
      m_flags &= ~TOUCHING_FLAG;
    }

    if (listener == null) {
      return;
    }

    if (wasTouching == false && touching == true) {
      listener.beginContact(this);
    }

    if (wasTouching == true && touching == false) {
      listener.endContact(this);
    }

    if (sensor == false && touching) {
      listener.preSolve(this, oldManifold);
    }
  }

  /**
   * Friction mixing law. The idea is to allow either fixture to drive the restitution to zero. For
   * example, anything slides on ice.
   * 
   * @param friction1
   * @param friction2
   * @return
   */
public static function mixFriction(friction1 : Float, friction2 : Float) : Float {
    return MathUtils.sqrt(friction1 * friction2);
  }

  /**
   * Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface. For
   * example, a superball bounces on anything.
   * 
   * @param restitution1
   * @param restitution2
   * @return
   */
public static function mixRestitution(restitution1 : Float, restitution2 : Float) : Float {
    return restitution1 > restitution2 ? restitution1 : restitution2;
  }
}

