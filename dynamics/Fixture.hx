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
package box2d.dynamics;

import box2d.collision.AABB;
import box2d.collision.RayCastInput;
import box2d.collision.RayCastOutput;
import box2d.collision.broadphase.BroadPhase;
import box2d.collision.shapes.MassData;
import box2d.collision.shapes.Shape;
import box2d.collision.shapes.ShapeType;
import box2d.common.MathUtils;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.contacts.Contact;
import box2d.dynamics.contacts.ContactEdge;

import haxe.ds.Vector;

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture inherits its
 * transform from its parent. Fixtures hold additional non-geometric data such as friction,
 * collision filters, etc. Fixtures are created via Body::CreateFixture.
 * 
 * @warning you cannot reuse fixtures.
 * 
 * @author daniel
 */
 class Fixture {

  public static var NULL_PROXY : Int = -1;

  public var m_density : Float;

  public var m_next : Fixture;
  public var m_body : Body;

  public var m_shape : Shape;

  public var m_friction : Float;
  public var m_restitution : Float;

  public var m_proxies : Vector<FixtureProxy>;
  public var m_proxyCount : Int;

  public var m_filter : Filter;

  public var m_isSensor : Bool;

  public var m_userData : Dynamic;

  public function new() {
    m_userData = null;
    m_body = null;
    m_next = null;
    m_proxies = null;
    m_proxyCount = 0;
    m_shape = null;
    m_filter = new Filter();
  }

  /**
   * Get the type of the child shape. You can use this to down cast to the concrete shape.
   * 
   * @return the shape type.
   */
  public function getType() : ShapeType {
    return m_shape.getType();
  }

  /**
   * Get the child shape. You can modify the child shape, however you should not change the number
   * of vertices because this will crash some collision caching mechanisms.
   * 
   * @return
   */
  public function getShape() : Shape {
    return m_shape;
  }

  /**
   * Is this fixture a sensor (non-solid)?
   * 
   * @return the true if the shape is a sensor.
   * @return
   */
  public function isSensor() : Bool {
    return m_isSensor;
  }

  /**
   * Set if this fixture is a sensor.
   * 
   * @param sensor
   */
  public function setSensor(sensor : Bool) : Void {
    if (sensor != m_isSensor) {
      m_body.setAwake(true);
      m_isSensor = sensor;
    }
  }

  /**
   * Set the contact filtering data. This is an expensive operation and should not be called
   * frequently. This will not update contacts until the next time step when either parent body is
   * awake. This automatically calls refilter.
   * 
   * @param filter
   */
  public function setFilterData(filter : Filter) : Void {
    m_filter.set(filter);

    refilter();
  }

  /**
   * Get the contact filtering data.
   * 
   * @return
   */
  public function getFilterData() : Filter {
    return m_filter;
  }

  /**
   * Call this if you want to establish collision that was previously disabled by
   * ContactFilter::ShouldCollide.
   */
  public function refilter() : Void {
    if (m_body == null) {
      return;
    }

    // Flag associated contacts for filtering.
    var edge : ContactEdge = m_body.getContactList();
    while (edge != null) {
      var contact : Contact = edge.contact;
      var fixtureA : Fixture = contact.getFixtureA();
      var fixtureB : Fixture = contact.getFixtureB();
      if (fixtureA == this || fixtureB == this) {
        contact.flagForFiltering();
      }
      edge = edge.next;
    }

    var world : World = m_body.getWorld();

    if (world == null) {
      return;
    }

    // Touch each proxy so that new pairs may be created
    var broadPhase : BroadPhase = world.m_contactManager.m_broadPhase;
    for (i in 0 ... m_proxyCount) {
    // for (int i = 0; i < m_proxyCount; ++i) {
      broadPhase.touchProxy(m_proxies[i].proxyId);
    }
  }

  /**
   * Get the parent body of this fixture. This is NULL if the fixture is not attached.
   * 
   * @return the parent body.
   * @return
   */
  public function getBody() : Body {
    return m_body;
  }

  /**
   * Get the next fixture in the parent body's fixture list.
   * 
   * @return the next shape.
   * @return
   */
  public function getNext() : Fixture {
    return m_next;
  }

  public function setDensity(density : Float) : Void {
    m_density = density;
  }

  public function getDensity() : Float {
    return m_density;
  }

  /**
   * Get the user data that was assigned in the fixture definition. Use this to store your
   * application specific data.
   * 
   * @return
   */
  public function getUserData() : Dynamic {
    return m_userData;
  }

  /**
   * Set the user data. Use this to store your application specific data.
   * 
   * @param data
   */
  public function setUserData(data : Dynamic) : Void {
    m_userData = data;
  }

  /**
   * Test a point for containment in this fixture. This only works for convex shapes.
   * 
   * @param p a point in world coordinates.
   * @return
   */
  public function testPoint(p : Vec2) : Bool {
    return m_shape.testPoint(m_body.m_xf, p);
  }

  /**
   * Cast a ray against this shape.
   * 
   * @param output the ray-cast results.
   * @param input the ray-cast input parameters.
   * @param output
   * @param input
   */
  public function raycast(output : RayCastOutput, input : RayCastInput, childIndex : Int) : Bool {
    return m_shape.raycast(output, input, m_body.m_xf, childIndex);
  }

  /**
   * Get the mass data for this fixture. The mass data is based on the density and the shape. The
   * rotational inertia is about the shape's origin.
   * 
   * @return
   */
  public function getMassData(massData : MassData) : Void {
    m_shape.computeMass(massData, m_density);
  }

  /**
   * Get the coefficient of friction.
   * 
   * @return
   */
  public function getFriction() : Float {
    return m_friction;
  }

  /**
   * Set the coefficient of friction. This will _not_ change the friction of existing contacts.
   * 
   * @param friction
   */
  public function setFriction(friction : Float) : Void {
    m_friction = friction;
  }

  /**
   * Get the coefficient of restitution.
   * 
   * @return
   */
  public function getRestitution() : Float {
    return m_restitution;
  }

  /**
   * Set the coefficient of restitution. This will _not_ change the restitution of existing
   * contacts.
   * 
   * @param restitution
   */
  public function setRestitution(restitution : Float) : Void {
    m_restitution = restitution;
  }

  /**
   * Get the fixture's AABB. This AABB may be enlarge and/or stale. If you need a more accurate
   * AABB, compute it using the shape and the body transform.
   * 
   * @return
   */
  public function getAABB(childIndex : Int) : AABB {
    return m_proxies[childIndex].aabb;
  }

  /**
   * Compute the distance from this fixture.
   * 
   * @param p a point in world coordinates.
   * @return distance
   */
  public function computeDistance(p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    return m_shape.computeDistanceToOut(m_body.getTransform(), p, childIndex, normalOut);
  }

  // We need separation create/destroy functions from the constructor/destructor because
  // the destructor cannot access the allocator (no destructor arguments allowed by C++).

  public function create(body : Body, def : FixtureDef) : Void {
    m_userData = def.userData;
    m_friction = def.friction;
    m_restitution = def.restitution;

    m_body = body;
    m_next = null;


    m_filter.set(def.filter);

    m_isSensor = def.isSensor;

    m_shape = def.shape.clone();

    // Reserve proxy space
    var childCount : Int = m_shape.getChildCount();
    if (m_proxies == null) {
      m_proxies = new Vector<FixtureProxy>(childCount);
      for(i in 0 ... childCount) {
        m_proxies[i] = new FixtureProxy();
        m_proxies[i].fixture = null;
        m_proxies[i].proxyId = NULL_PROXY;
      }
    }

    if (m_proxies.length < childCount) {
      var old : Vector<FixtureProxy> = m_proxies;
      var newLen : Int = MathUtils.max(old.length * 2, childCount);
      m_proxies = new Vector<FixtureProxy>(newLen);
      // TODO: array copy
      Vector.blit(old, 0, m_proxies, 0, old.length);
      // System.arraycopy(old, 0, m_proxies, 0, old.length);
      for(i in 0 ... newLen) {
        if (i >= old.length) {
          m_proxies[i] = new FixtureProxy();
        }
        m_proxies[i].fixture = null;
        m_proxies[i].proxyId = NULL_PROXY;
      }
    }
    m_proxyCount = 0;

    m_density = def.density;
  }

  public function destroy() : Void {
    // The proxies must be destroyed before calling this.

    // Free the child shape.
    m_shape = null;
    m_proxies = null;
    m_next = null;

    // TODO pool shapes
    // TODO pool fixtures
  }

  // These support body activation/deactivation.
  public function createProxies(broadPhase : BroadPhase, xf : Transform) : Void {

    // Create proxies in the broad-phase.
    m_proxyCount = m_shape.getChildCount();

    for (i in 0 ... m_proxyCount) {
    // for (int i = 0; i < m_proxyCount; ++i) {
      var proxy : FixtureProxy = m_proxies[i];
      m_shape.computeAABB(proxy.aabb, xf, i);
      proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy);
      proxy.fixture = this;
      proxy.childIndex = i;
    }
  }

  /**
   * Internal method
   * 
   * @param broadPhase
   */
  public function destroyProxies(broadPhase : BroadPhase) : Void {
    // Destroy proxies in the broad-phase.
    for (i in 0 ... m_proxyCount) {
    // for (int i = 0; i < m_proxyCount; ++i) {
      var proxy : FixtureProxy = m_proxies[i];
      broadPhase.destroyProxy(proxy.proxyId);
      proxy.proxyId = NULL_PROXY;
    }

    m_proxyCount = 0;
  }

  private var pool1 : AABB = new AABB();
  private var pool2 : AABB = new AABB();
  private var displacement : Vec2 = new Vec2();

  /**
   * Internal method
   * 
   * @param broadPhase
   * @param xf1
   * @param xf2
   */
  public function synchronize(broadPhase : BroadPhase, transform1 : Transform, transform2 : Transform) : Void {
    if (m_proxyCount == 0) {
      return;
    }

    for (i in 0 ... m_proxyCount) {
    // for (int i = 0; i < m_proxyCount; ++i) {
      var proxy : FixtureProxy = m_proxies[i];

      // Compute an AABB that covers the swept shape (may miss some rotation effect).
      var aabb1 : AABB = pool1;
      var aab : AABB = pool2;
      m_shape.computeAABB(aabb1, transform1, proxy.childIndex);
      m_shape.computeAABB(aab, transform2, proxy.childIndex);

      proxy.aabb.lowerBound.x =
          aabb1.lowerBound.x < aab.lowerBound.x ? aabb1.lowerBound.x : aab.lowerBound.x;
      proxy.aabb.lowerBound.y =
          aabb1.lowerBound.y < aab.lowerBound.y ? aabb1.lowerBound.y : aab.lowerBound.y;
      proxy.aabb.upperBound.x =
          aabb1.upperBound.x > aab.upperBound.x ? aabb1.upperBound.x : aab.upperBound.x;
      proxy.aabb.upperBound.y =
          aabb1.upperBound.y > aab.upperBound.y ? aabb1.upperBound.y : aab.upperBound.y;
      displacement.x = transform2.p.x - transform1.p.x;
      displacement.y = transform2.p.y - transform1.p.y;

      broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement);
    }
  }
}

