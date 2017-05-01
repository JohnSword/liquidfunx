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

import box2d.callbacks.ContactFilter;
import box2d.callbacks.ContactListener;
import box2d.callbacks.DebugDraw;
import box2d.callbacks.DestructionListener;
import box2d.callbacks.ParticleDestructionListener;
import box2d.callbacks.ParticleQueryCallback;
import box2d.callbacks.ParticleRaycastCallback;
import box2d.callbacks.QueryCallback;
import box2d.callbacks.RayCastCallback;
import box2d.callbacks.TreeCallback;
import box2d.callbacks.TreeRayCastCallback;
import box2d.collision.AABB;
import box2d.collision.RayCastInput;
import box2d.collision.RayCastOutput;
import box2d.collision.TOIInput;
import box2d.collision.TOIOutput;
import box2d.collision.TimeOfImpact.TOIOutputState;
import box2d.collision.broadphase.BroadPhase;
import box2d.collision.broadphase.BroadPhaseStrategy;
import box2d.collision.broadphase.DefaultBroadPhaseBuffer;
import box2d.collision.broadphase.DynamicTree;
import box2d.collision.shapes.ChainShape;
import box2d.collision.shapes.CircleShape;
import box2d.collision.shapes.EdgeShape;
import box2d.collision.shapes.PolygonShape;
import box2d.collision.shapes.Shape;
import box2d.collision.shapes.ShapeType;
import box2d.common.Color3f;
import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Sweep;
import box2d.common.TimerB2d;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.contacts.Contact;
import box2d.dynamics.contacts.ContactEdge;
import box2d.dynamics.contacts.ContactRegister;
import box2d.dynamics.joints.Joint;
import box2d.dynamics.joints.JointDef;
import box2d.dynamics.joints.JointEdge;
import box2d.dynamics.joints.PulleyJoint;
import box2d.particle.ParticleBodyContact;
import box2d.particle.ParticleColor;
import box2d.particle.ParticleContact;
import box2d.particle.ParticleDef;
import box2d.particle.ParticleGroup;
import box2d.particle.ParticleGroupDef;
import box2d.particle.ParticleSystem;
import box2d.pooling.IDynamicStack;
import box2d.pooling.IWorldPool;
import box2d.pooling.arrays.Vec2Array;
import box2d.pooling.normal.DefaultWorldPool;

import haxe.ds.Vector;

/**
 * The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
 * world also contains efficient memory management facilities.
 * 
 * @author Daniel Murphy
 */
 class World {
  public static var WORLD_POOL_SIZE : Int = 100;
  public static var WORLD_POOL_CONTAINER_SIZE : Int = 10;

  public static var NEW_FIXTURE : Int = 0x0001;
  public static var LOCKED : Int = 0x0002;
  public static var CLEAR_FORCES : Int = 0x0004;


  // statistics gathering
  public var activeContacts : Int = 0;
  public var contactPoolCount : Int = 0;

  public var m_flags : Int;

  public var m_contactManager : ContactManager;

  private var m_bodyList : Body;
  private var m_jointList : Joint;

  private var m_bodyCount : Int;
  private var m_jointCount : Int;

  private var m_gravity : Vec2 = new Vec2();
  private var m_allowSleep : Bool;

  // private Body m_groundBody;

  private var m_destructionListener : DestructionListener;
  private var m_particleDestructionListener : ParticleDestructionListener;
  private var m_debugDraw : DebugDraw;

  private var pool : IWorldPool;

  /**
   * This is used to compute the time step ratio to support a variable time step.
   */
  private var m_inv_dt0 : Float;

  // these are for debugging the solver
  private var m_warmStarting : Bool;
  private var m_continuousPhysics : Bool;
  private var m_subStepping : Bool;

  private var m_stepComplete : Bool;

  private var m_profile : Profile;

  private var m_particleSystem : ParticleSystem;


  // private ContactRegister[][] contactStacks =
  //     new ContactRegister[ShapeType.values().length][ShapeType.values().length];
  // TODO: ShapeType does not have length property
  private var contactStacks:Array<Array<ContactRegister>> = cast [for (x in 0...4) [for (y in 0...4) 0]];

  /**
   * Construct a world object.
   * 
   * @param gravity the world gravity vector.
   */
  public function new(gravity : Vec2) {
    new2(gravity, new DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE));
  }

  /**
   * Construct a world object.
   * 
   * @param gravity the world gravity vector.
   */
  public function new2(gravity : Vec2, pool : IWorldPool) {
    new3(gravity, pool, new DynamicTree());
  }

  public function new3(gravity : Vec2, pool : IWorldPool, strategy : BroadPhaseStrategy) {
    new4(gravity, pool, new DefaultBroadPhaseBuffer(strategy));
  }

  public function new4(gravity : Vec2, pool : IWorldPool, broadPhase : BroadPhase) {
    this.pool = pool;
    m_destructionListener = null;
    m_debugDraw = null;

    m_bodyList = null;
    m_jointList = null;

    m_bodyCount = 0;
    m_jointCount = 0;

    m_warmStarting = true;
    m_continuousPhysics = true;
    m_subStepping = false;
    m_stepComplete = true;

    m_allowSleep = true;
    m_gravity.setVec(gravity);

    m_flags = CLEAR_FORCES;

    m_inv_dt0 = 0;

    m_contactManager = new ContactManager(this, broadPhase);
    m_profile = new Profile();

    m_particleSystem = new ParticleSystem(this);

    initializeRegisters();
  }

  public function setAllowSleep(flag : Bool) : Void {
    if (flag == m_allowSleep) {
      return;
    }

    m_allowSleep = flag;
    if (m_allowSleep == false) {
      var b : Body = m_bodyList;
      while (b != null) {
        b.setAwake(true);
        b = b.m_next;
      }
    }
  }

  public function setSubStepping(subStepping : Bool) : Void {
    this.m_subStepping = subStepping;
  }

  public function isSubStepping() : Bool {
    return m_subStepping;
  }

  public function isAllowSleep() : Bool {
    return m_allowSleep;
  }

  private function addType(creator : IDynamicStack<Contact>, type1 : ShapeType, type2 : ShapeType) : Void {
    var register : ContactRegister = new ContactRegister();
    register.creator = creator;
    register.primary = true;
    contactStacks[type1.getIndex()][type2.getIndex()] = register;

    if (type1 != type2) {
      var register2 : ContactRegister = new ContactRegister();
      register2.creator = creator;
      register2.primary = false;
      contactStacks[type2.getIndex()][type1.getIndex()] = register2;
    }
  }

  private function initializeRegisters() : Void {
    addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE);
    addType(pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE);
    addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON);
    addType(pool.getEdgeCircleContactStack(), ShapeType.EDGE, ShapeType.CIRCLE);
    addType(pool.getEdgePolyContactStack(), ShapeType.EDGE, ShapeType.POLYGON);
    addType(pool.getChainCircleContactStack(), ShapeType.CHAIN, ShapeType.CIRCLE);
    addType(pool.getChainPolyContactStack(), ShapeType.CHAIN, ShapeType.POLYGON);
  }

  public function getDestructionListener() : DestructionListener {
    return m_destructionListener;
  }

  public function getParticleDestructionListener() : ParticleDestructionListener {
    return m_particleDestructionListener;
  }

  public function setParticleDestructionListener(listener : ParticleDestructionListener) : Void {
    m_particleDestructionListener = listener;
  }

  public function popContact(fixtureA : Fixture, indexA : Int, fixtureB : Fixture, indexB : Int) : Contact {
    var type1 : ShapeType = fixtureA.getType();
    var type2 : ShapeType = fixtureB.getType();

    var reg : ContactRegister = contactStacks[type1.getIndex()][type2.getIndex()];
    if (reg != null) {
      if (reg.primary) {
        var c : Contact = reg.creator.pop();
        c.init(fixtureA, indexA, fixtureB, indexB);
        return c;
      } else {
        var c : Contact = reg.creator.pop();
        c.init(fixtureB, indexB, fixtureA, indexA);
        return c;
      }
    } else {
      return null;
    }
  }

  public function pushContact(contact : Contact) : Void {
    var fixtureA : Fixture = contact.getFixtureA();
    var fixtureB : Fixture = contact.getFixtureB();

    if (contact.m_manifold.pointCount > 0 && !fixtureA.isSensor() && !fixtureB.isSensor()) {
      fixtureA.getBody().setAwake(true);
      fixtureB.getBody().setAwake(true);
    }

    var type1 : ShapeType = fixtureA.getType();
    var type2 : ShapeType = fixtureB.getType();

    var creator : IDynamicStack<Contact> = contactStacks[type1.getIndex()][type2.getIndex()].creator;
    creator.push(contact);
  }

  public function getPool() : IWorldPool {
    return pool;
  }

  /**
   * Register a destruction listener. The listener is owned by you and must remain in scope.
   * 
   * @param listener
   */
  public function setDestructionListener(listener : DestructionListener) : Void {
    m_destructionListener = listener;
  }

  /**
   * Register a contact filter to provide specific control over collision. Otherwise the default
   * filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
   * 
   * @param filter
   */
  public function setContactFilter(filter : ContactFilter) : Void {
    m_contactManager.m_contactFilter = filter;
  }

  /**
   * Register a contact event listener. The listener is owned by you and must remain in scope.
   * 
   * @param listener
   */
  public function setContactListener(listener : ContactListener) : Void {
    m_contactManager.m_contactListener = listener;
  }

  /**
   * Register a routine for debug drawing. The debug draw functions are called inside with
   * World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
   * 
   * @param debugDraw
   */
  public function setDebugDraw(debugDraw : DebugDraw) : Void {
    m_debugDraw = debugDraw;
  }

  /**
   * create a rigid body given a definition. No reference to the definition is retained.
   * 
   * @warning This function is locked during callbacks.
   * @param def
   * @return
   */
  public function createBody(def : BodyDef) : Body {
    if (isLocked()) {
      return null;
    }
    // TODO djm pooling
    var b : Body = new Body(def, this);

    // add to world doubly linked list
    b.m_prev = null;
    b.m_next = m_bodyList;
    if (m_bodyList != null) {
      m_bodyList.m_prev = b;
    }
    m_bodyList = b;
    ++m_bodyCount;

    return b;
  }

  /**
   * destroy a rigid body given a definition. No reference to the definition is retained. This
   * function is locked during callbacks.
   * 
   * @warning This automatically deletes all associated shapes and joints.
   * @warning This function is locked during callbacks.
   * @param body
   */
  public function destroyBody(body : Body) : Void {
    if (isLocked()) {
      return;
    }

    // Delete the attached joints.
    var je : JointEdge = body.m_jointList;
    while (je != null) {
      var je0 : JointEdge = je;
      je = je.next;
      if (m_destructionListener != null) {
        m_destructionListener.sayGoodbyeJoint(je0.joint);
      }

      destroyJoint(je0.joint);

      body.m_jointList = je;
    }
    body.m_jointList = null;

    // Delete the attached contacts.
    var ce : ContactEdge = body.m_contactList;
    while (ce != null) {
      var ce0 : ContactEdge = ce;
      ce = ce.next;
      m_contactManager.destroy(ce0.contact);
    }
    body.m_contactList = null;

    var f : Fixture = body.m_fixtureList;
    while (f != null) {
      var f0 : Fixture = f;
      f = f.m_next;

      if (m_destructionListener != null) {
        m_destructionListener.sayGoodbyeFixture(f0);
      }

      f0.destroyProxies(m_contactManager.m_broadPhase);
      f0.destroy();
      // TODO djm recycle fixtures (here or in that destroy method)
      body.m_fixtureList = f;
      body.m_fixtureCount -= 1;
    }
    body.m_fixtureList = null;
    body.m_fixtureCount = 0;

    // Remove world body list.
    if (body.m_prev != null) {
      body.m_prev.m_next = body.m_next;
    }

    if (body.m_next != null) {
      body.m_next.m_prev = body.m_prev;
    }

    if (body == m_bodyList) {
      m_bodyList = body.m_next;
    }

    --m_bodyCount;
    // TODO djm recycle body
  }

  /**
   * create a joint to constrain bodies together. No reference to the definition is retained. This
   * may cause the connected bodies to cease colliding.
   * 
   * @warning This function is locked during callbacks.
   * @param def
   * @return
   */
  public function createJoint(def : JointDef) : Joint {
    if (isLocked()) {
      return null;
    }

    var j : Joint = Joint.create(this, def);

    // Connect to the world list.
    j.m_prev = null;
    j.m_next = m_jointList;
    if (m_jointList != null) {
      m_jointList.m_prev = j;
    }
    m_jointList = j;
    ++m_jointCount;

    // Connect to the bodies' doubly linked lists.
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.getBodyB();
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.getBodyA().m_jointList;
    if (j.getBodyA().m_jointList != null) {
      j.getBodyA().m_jointList.prev = j.m_edgeA;
    }
    j.getBodyA().m_jointList = j.m_edgeA;

    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.getBodyA();
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.getBodyB().m_jointList;
    if (j.getBodyB().m_jointList != null) {
      j.getBodyB().m_jointList.prev = j.m_edgeB;
    }
    j.getBodyB().m_jointList = j.m_edgeB;

    var bodyA : Body = def.bodyA;
    var bodyB : Body = def.bodyB;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (def.collideConnected == false) {
      var edge : ContactEdge = bodyB.getContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.flagForFiltering();
        }

        edge = edge.next;
      }
    }

    // Note: creating a joint doesn't wake the bodies.

    return j;
  }

  /**
   * destroy a joint. This may cause the connected bodies to begin colliding.
   * 
   * @warning This function is locked during callbacks.
   * @param joint
   */
  public function destroyJoint(j : Joint) : Void {
    if (isLocked()) {
      return;
    }

    var collideConnected : Bool = j.getCollideConnected();

    // Remove from the doubly linked list.
    if (j.m_prev != null) {
      j.m_prev.m_next = j.m_next;
    }

    if (j.m_next != null) {
      j.m_next.m_prev = j.m_prev;
    }

    if (j == m_jointList) {
      m_jointList = j.m_next;
    }

    // Disconnect from island graph.
    var bodyA : Body = j.getBodyA();
    var bodyB : Body = j.getBodyB();

    // Wake up connected bodies.
    bodyA.setAwake(true);
    bodyB.setAwake(true);

    // Remove from body 1.
    if (j.m_edgeA.prev != null) {
      j.m_edgeA.prev.next = j.m_edgeA.next;
    }

    if (j.m_edgeA.next != null) {
      j.m_edgeA.next.prev = j.m_edgeA.prev;
    }

    if (j.m_edgeA == bodyA.m_jointList) {
      bodyA.m_jointList = j.m_edgeA.next;
    }

    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;

    // Remove from body 2
    if (j.m_edgeB.prev != null) {
      j.m_edgeB.prev.next = j.m_edgeB.next;
    }

    if (j.m_edgeB.next != null) {
      j.m_edgeB.next.prev = j.m_edgeB.prev;
    }

    if (j.m_edgeB == bodyB.m_jointList) {
      bodyB.m_jointList = j.m_edgeB.next;
    }

    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;

    Joint.destroy(j);

    --m_jointCount;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (collideConnected == false) {
      var edge : ContactEdge = bodyB.getContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.flagForFiltering();
        }

        edge = edge.next;
      }
    }
  }

  // djm pooling
  private var timestep : TimeStep = new TimeStep();
  private var stepTimer : TimerB2d = new box2d.common.TimerB2d();
  private var tempTimer : TimerB2d = new box2d.common.TimerB2d();

  /**
   * Take a time step. This performs collision detection, integration, and constraint solution.
   * 
   * @param timeStep the amount of time to simulate, this should not vary.
   * @param velocityIterations for the velocity constraint solver.
   * @param positionIterations for the position constraint solver.
   */
  public function step(dt : Float, velocityIterations : Int, positionIterations : Int) : Void {
    stepTimer.reset();
    tempTimer.reset();
    // log.debug("Starting step");
    // If new fixtures were added, we need to find the new contacts.
    if ((m_flags & NEW_FIXTURE) == NEW_FIXTURE) {
      // log.debug("There's a new fixture, lets look for new contacts");
      m_contactManager.findNewContacts();
      m_flags &= ~NEW_FIXTURE;
    }

    m_flags |= LOCKED;

    timestep.dt = dt;
    timestep.velocityIterations = velocityIterations;
    timestep.positionIterations = positionIterations;
    if (dt > 0.0) {
      timestep.inv_dt = 1.0 / dt;
    } else {
      timestep.inv_dt = 0.0;
    }

    timestep.dtRatio = m_inv_dt0 * dt;

    timestep.warmStarting = m_warmStarting;
    m_profile.stepInit.record(tempTimer.getMilliseconds());

    // Update contacts. This is where some contacts are destroyed.
    tempTimer.reset();
    m_contactManager.collide();
    m_profile.collide.record(tempTimer.getMilliseconds());

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (m_stepComplete && timestep.dt > 0.0) {
      tempTimer.reset();
      m_particleSystem.solve(timestep); // Particle Simulation
      m_profile.solveParticleSystem.record(tempTimer.getMilliseconds());
      tempTimer.reset();
      solve(timestep);
      m_profile.solve.record(tempTimer.getMilliseconds());
    }

    // Handle TOI events.
    if (m_continuousPhysics && timestep.dt > 0.0) {
      tempTimer.reset();
      solveTOI(timestep);
      m_profile.solveTOI.record(tempTimer.getMilliseconds());
    }

    if (timestep.dt > 0.0) {
      m_inv_dt0 = timestep.inv_dt;
    }

    if ((m_flags & CLEAR_FORCES) == CLEAR_FORCES) {
      clearForces();
    }

    m_flags &= ~LOCKED;
    // log.debug("ending step");

    m_profile.step.record(stepTimer.getMilliseconds());
  }

  /**
   * Call this after you are done with time steps to clear the forces. You normally call this after
   * each call to Step, unless you are performing sub-steps. By default, forces will be
   * automatically cleared, so you don't need to call this function.
   * 
   * @see setAutoClearForces
   */
  public function clearForces() : Void {
    var body : Body = m_bodyList;
    while (body != null) {
      body.m_force.setZero();
      body.m_torque = 0.0;
      body = body.getNext();
    }
  }

  private var color : Color3f = new Color3f();
  private var xf : Transform = new Transform();
  private var cA : Vec2 = new Vec2();
  private var cB : Vec2 = new Vec2();
  private var avs : Vec2Array = new Vec2Array();

  /**
   * Call this to draw shapes and other debug draw data.
   */
  public function drawDebugData() : Void {
    if (m_debugDraw == null) {
      return;
    }


    var flags : Int = m_debugDraw.getFlags();
    var wireframe : Bool = (flags & DebugDraw.e_wireframeDrawingBit) != 0;

    if ((flags & DebugDraw.e_shapeBit) != 0) {
      var b : Body = m_bodyList;
      while (b != null) {
        xf.set(b.getTransform());
        var f : Fixture = b.getFixtureList();
        while (f != null) {
          if (b.isActive() == false) {
            color.set(0.5, 0.5, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.getType() == BodyType.STATIC) {
            color.set(0.5, 0.9, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.getType() == BodyType.KINEMATIC) {
            color.set(0.5, 0.5, 0.9);
            drawShape(f, xf, color, wireframe);
          } else if (b.isAwake() == false) {
            color.set(0.5, 0.5, 0.5);
            drawShape(f, xf, color, wireframe);
          } else {
            color.set(0.9, 0.7, 0.7);
            drawShape(f, xf, color, wireframe);
          }
          f = f.getNext();
        }
        b = b.getNext();
      }
      drawParticleSystem(m_particleSystem);
    }

    if ((flags & DebugDraw.e_jointBit) != 0) {
      var j : Joint = m_jointList;
      while (j != null) {
        drawJoint(j);
        j = j.getNext();
      }
    }

    if ((flags & DebugDraw.e_pairBit) != 0) {
      color.set(0.3, 0.9, 0.9);
      var c : Contact = m_contactManager.m_contactList;
      while (c != null) {
        var fixtureA : Fixture = c.getFixtureA();
        var fixtureB : Fixture = c.getFixtureB();
        fixtureA.getAABB(c.getChildIndexA()).getCenterToOut(cA);
        fixtureB.getAABB(c.getChildIndexB()).getCenterToOut(cB);
        m_debugDraw.drawSegment(cA, cB, color);
        c = c.getNext();
      }
    }

    if ((flags & DebugDraw.e_aabbBit) != 0) {
      color.set(0.9, 0.3, 0.9);

      var b : Body = m_bodyList;
      while (b != null) {
        if (b.isActive() == false) {
          b = b.getNext();
          continue;
        }

        var f : Fixture = b.getFixtureList();
        while (f != null) {
          for (i in 0 ... f.m_proxyCount) {
          // for (int i = 0; i < f.m_proxyCount; ++i) {
            var proxy : FixtureProxy = f.m_proxies[i];
            var aabb : AABB = m_contactManager.m_broadPhase.getFatAABB(proxy.proxyId);
            if (aabb != null) {
              var vs : Vector<Vec2> = avs.get(4);
              vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y);
              vs[1].set(aabb.upperBound.x, aabb.lowerBound.y);
              vs[2].set(aabb.upperBound.x, aabb.upperBound.y);
              vs[3].set(aabb.lowerBound.x, aabb.upperBound.y);
              m_debugDraw.drawPolygon(vs, 4, color);
            }
          }
          f = f.getNext();
        }
      }
      b = b.getNext();
    }

    if ((flags & DebugDraw.e_centerOfMassBit) != 0) {
      var b : Body = m_bodyList;
      while (b != null) {
        xf.set(b.getTransform());
        xf.p.setVec(b.getWorldCenter());
        m_debugDraw.drawTransform(xf);
        b = b.getNext();
      }
    }

    if ((flags & DebugDraw.e_dynamicTreeBit) != 0) {
      m_contactManager.m_broadPhase.drawTree(m_debugDraw);
    }

    m_debugDraw.flush();
  }

  private var wqwrapper : WorldQueryWrapper = new WorldQueryWrapper();

  /**
   * Query the world for all fixtures that potentially overlap the provided AABB.
   * 
   * @param callback a user implemented callback class.
   * @param aabb the query box.
   */
  public function queryAABB(callback : QueryCallback, aabb : AABB) : Void {
    wqwrapper.broadPhase = m_contactManager.m_broadPhase;
    wqwrapper.callback = callback;
    m_contactManager.m_broadPhase.query(wqwrapper, aabb);
  }

  /**
   * Query the world for all fixtures and particles that potentially overlap the provided AABB.
   * 
   * @param callback a user implemented callback class.
   * @param particleCallback callback for particles.
   * @param aabb the query box.
   */
  public function queryParticleAABB(callback : QueryCallback, particleCallback : ParticleQueryCallback, aabb : AABB) : Void {
    wqwrapper.broadPhase = m_contactManager.m_broadPhase;
    wqwrapper.callback = callback;
    m_contactManager.m_broadPhase.query(wqwrapper, aabb);
    m_particleSystem.queryAABB(particleCallback, aabb);
  }

  /**
   * Query the world for all particles that potentially overlap the provided AABB.
   * 
   * @param particleCallback callback for particles.
   * @param aabb the query box.
   */
  public function queryParticleAABB2(particleCallback : ParticleQueryCallback, aabb : AABB) : Void {
    m_particleSystem.queryAABB(particleCallback, aabb);
  }

  private var wrcwrapper : WorldRayCastWrapper = new WorldRayCastWrapper();
  private var input : RayCastInput = new RayCastInput();

  /**
   * Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
   * get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
   * starting point.
   * 
   * @param callback a user implemented callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  public function raycast(callback : RayCastCallback, point1 : Vec2, point2 : Vec2) : Void {
    wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setVec(point1);
    input.p2.setVec(point2);
    m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
  }

  /**
   * Ray-cast the world for all fixtures and particles in the path of the ray. Your callback
   * controls whether you get the closest point, any point, or n-points. The ray-cast ignores shapes
   * that contain the starting point.
   * 
   * @param callback a user implemented callback class.
   * @param particleCallback the particle callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  public function raycastParticle(callback : RayCastCallback, particleCallback : ParticleRaycastCallback, point1 : Vec2, point2 : Vec2) : Void {
    wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setVec(point1);
    input.p2.setVec(point2);
    m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
    m_particleSystem.raycast(particleCallback, point1, point2);
  }

  /**
   * Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
   * get the closest point, any point, or n-points.
   * 
   * @param particleCallback the particle callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  public function raycastPoint(particleCallback : ParticleRaycastCallback, point1 : Vec2, point2 : Vec2) : Void {
    m_particleSystem.raycast(particleCallback, point1, point2);
  }

  /**
   * Get the world body list. With the returned body, use Body.getNext to get the next body in the
   * world list. A null body indicates the end of the list.
   * 
   * @return the head of the world body list.
   */
  public function getBodyList() : Body {
    return m_bodyList;
  }

  /**
   * Get the world joint list. With the returned joint, use Joint.getNext to get the next joint in
   * the world list. A null joint indicates the end of the list.
   * 
   * @return the head of the world joint list.
   */
  public function getJointList() : Joint {
    return m_jointList;
  }

  /**
   * Get the world contact list. With the returned contact, use Contact.getNext to get the next
   * contact in the world list. A null contact indicates the end of the list.
   * 
   * @return the head of the world contact list.
   * @warning contacts are created and destroyed in the middle of a time step. Use ContactListener
   *          to avoid missing contacts.
   */
  public function getContactList() : Contact {
    return m_contactManager.m_contactList;
  }

  public function isSleepingAllowed() : Bool {
    return m_allowSleep;
  }

  public function setSleepingAllowed(sleepingAllowed : Bool) : Void {
    m_allowSleep = sleepingAllowed;
  }

  /**
   * Enable/disable warm starting. For testing.
   * 
   * @param flag
   */
  public function setWarmStarting(flag : Bool) : Void {
    m_warmStarting = flag;
  }

  public function isWarmStarting() : Bool {
    return m_warmStarting;
  }

  /**
   * Enable/disable continuous physics. For testing.
   * 
   * @param flag
   */
  public function setContinuousPhysics(flag : Bool) : Void {
    m_continuousPhysics = flag;
  }

  public function isContinuousPhysics() : Bool {
    return m_continuousPhysics;
  }



  /**
   * Get the number of broad-phase proxies.
   * 
   * @return
   */
  public function getProxyCount() : Int {
    return m_contactManager.m_broadPhase.getProxyCount();
  }

  /**
   * Get the number of bodies.
   * 
   * @return
   */
  public function getBodyCount() : Int {
    return m_bodyCount;
  }

  /**
   * Get the number of joints.
   * 
   * @return
   */
  public function getJointCount() : Int {
    return m_jointCount;
  }

  /**
   * Get the number of contacts (each may have 0 or more contact points).
   * 
   * @return
   */
  public function getContactCount() : Int {
    return m_contactManager.m_contactCount;
  }

  /**
   * Gets the height of the dynamic tree
   * 
   * @return
   */
  public function getTreeHeight() : Int {
    return m_contactManager.m_broadPhase.getTreeHeight();
  }

  /**
   * Gets the balance of the dynamic tree
   * 
   * @return
   */
  public function getTreeBalance() : Int {
    return m_contactManager.m_broadPhase.getTreeBalance();
  }

  /**
   * Gets the quality of the dynamic tree
   * 
   * @return
   */
  public function getTreeQuality() : Float {
    return m_contactManager.m_broadPhase.getTreeQuality();
  }

  /**
   * Change the global gravity vector.
   * 
   * @param gravity
   */
  public function setGravity(gravity : Vec2) : Void {
    m_gravity.setVec(gravity);
  }

  /**
   * Get the global gravity vector.
   * 
   * @return
   */
  public function getGravity() : Vec2 {
    return m_gravity;
  }

  /**
   * Is the world locked (in the middle of a time step).
   * 
   * @return
   */
  public function isLocked() : Bool {
    return (m_flags & LOCKED) == LOCKED;
  }

  /**
   * Set flag to control automatic clearing of forces after each time step.
   * 
   * @param flag
   */
  public function setAutoClearForces(flag : Bool) : Void {
    if (flag) {
      m_flags |= CLEAR_FORCES;
    } else {
      m_flags &= ~CLEAR_FORCES;
    }
  }

  /**
   * Get the flag that controls automatic clearing of forces after each time step.
   * 
   * @return
   */
  public function getAutoClearForces() : Bool {
    return (m_flags & CLEAR_FORCES) == CLEAR_FORCES;
  }

  /**
   * Get the contact manager for testing purposes
   * 
   * @return
   */
  public function getContactManager() : ContactManager {
    return m_contactManager;
  }

  public function getProfile() : Profile {
    return m_profile;
  }

  private var island : Island = new Island();
  private var stack : Vector<Body> = new Vector<Body>(10); // TODO djm find a good initial stack number;
  private var broadphaseTimer : TimerB2d = new TimerB2d();

  private function solve(step : TimeStep) : Void {
    m_profile.solveInit.startAccum();
    m_profile.solveVelocity.startAccum();
    m_profile.solvePosition.startAccum();

    // update previous transforms
    var b : Body = m_bodyList;
    while (b != null) {
      b.m_xf0.set(b.m_xf);
      b = b.m_next;
    }

    // Size the island for the worst case.
    island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount,
        m_contactManager.m_contactListener);

    // Clear all the island flags.
    var b : Body = m_bodyList;
    while (b != null) {
      b.m_flags &= ~Body.e_islandFlag;
      b = b.m_next;
    }
    var c : Contact = m_contactManager.m_contactList;
    while (c != null) {
      c.m_flags &= ~Contact.ISLAND_FLAG;
      c = c.m_next;
    }
    var j : Joint = m_jointList;
    while (j != null) {
      j.m_islandFlag = false;
      j = j.m_next;
    }

    // Build and simulate all awake islands.
    var stackSize : Int = m_bodyCount;
    if (stack.length < stackSize) {
      stack = new Vector<Body>(stackSize);
    }
    var seed : Body = m_bodyList;
    while (seed != null) {
      if ((seed.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
        seed = seed.m_next;
        continue;
      }

      if (seed.isAwake() == false || seed.isActive() == false) {
        seed = seed.m_next;
        continue;
      }

      // The seed can be dynamic or kinematic.
      if (seed.getType() == BodyType.STATIC) {
        seed = seed.m_next;
        continue;
      }

      // Reset island and stack.
      island.clear();
      var stackCount : Int = 0;
      stack[stackCount++] = seed;
      seed.m_flags |= Body.e_islandFlag;

      // Perform a depth first search (DFS) on the constraint graph.
      while (stackCount > 0) {
        // Grab the next body off the stack and add it to the island.
        var b : Body = stack[--stackCount];
        island.addBody(b);

        // Make sure the body is awake.
        b.setAwake(true);

        // To keep islands as small as possible, we don't
        // propagate islands across static bodies.
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        // Search all contacts connected to this body.
        var ce : ContactEdge = b.m_contactList;
        while (ce != null) {
          var contact : Contact = ce.contact;

          // Has this contact already been added to an island?
          if ((contact.m_flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG) {
            ce = ce.next;
            continue;
          }

          // Is this contact solid and touching?
          if (contact.isEnabled() == false || contact.isTouching() == false) {
            ce = ce.next;
            continue;
          }

          // Skip sensors.
          var sensorA : Bool = contact.m_fixtureA.m_isSensor;
          var sensorB : Bool = contact.m_fixtureB.m_isSensor;
          if (sensorA || sensorB) {
            ce = ce.next;
            continue;
          }

          island.addContact(contact);
          contact.m_flags |= Contact.ISLAND_FLAG;

          var other : Body = ce.other;

          // Was the other body already added to this island?
          if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
            ce = ce.next;
            continue;
          }

          stack[stackCount++] = other;
          other.m_flags |= Body.e_islandFlag;

          ce = ce.next;
        }

        // Search all joints connect to this body.
        var je : JointEdge = b.m_jointList;
        while (je != null) {
          if (je.joint.m_islandFlag == true) {
            je = je.next;
            continue;
          }

          var other : Body = je.other;

          // Don't simulate joints connected to inactive bodies.
          if (other.isActive() == false) {
            je = je.next;
            continue;
          }

          island.addJoint(je.joint);
          je.joint.m_islandFlag = true;

          if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
            je = je.next;
            continue;
          }

          stack[stackCount++] = other;
          other.m_flags |= Body.e_islandFlag;
          je = je.next;
        }
      }
      island.solve(m_profile, step, m_gravity, m_allowSleep);

      // Post solve cleanup.
      for (i in 0 ... island.m_bodyCount) {
      // for (int i = 0; i < island.m_bodyCount; ++i) {
        // Allow static bodies to participate in other islands.
        var b : Body = island.m_bodies[i];
        if (b.getType() == BodyType.STATIC) {
          b.m_flags &= ~Body.e_islandFlag;
        }
      }

      seed = seed.m_next;
    }
    m_profile.solveInit.endAccum();
    m_profile.solveVelocity.endAccum();
    m_profile.solvePosition.endAccum();

    broadphaseTimer.reset();
    // Synchronize fixtures, check for out of range bodies.
    var b : Body = m_bodyList;
    while (b != null) {
      // If a body was not in an island then it did not move.
      if ((b.m_flags & Body.e_islandFlag) == 0) {
        b = b.m_next;
        continue;
      }

      if (b.getType() == BodyType.STATIC) {
        b = b.m_next;
        continue;
      }

      // Update fixtures (for broad-phase).
      b.synchronizeFixtures();
      b = b.m_next;
    }

    // Look for new contacts.
    m_contactManager.findNewContacts();
    m_profile.broadphase.record(broadphaseTimer.getMilliseconds());
  }

  private var toiIsland : Island = new Island();
  private var toiInput : TOIInput = new TOIInput();
  private var toiOutput : TOIOutput = new TOIOutput();
  private var subStep : TimeStep = new TimeStep();
  private var tempBodies : Array<Body> = new Array<Body>();
  private var backup1 : Sweep = new Sweep();
  private var backup2 : Sweep = new Sweep();

  private function solveTOI(step : TimeStep) : Void {

    var island : Island = toiIsland;
    island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0,
        m_contactManager.m_contactListener);
    if (m_stepComplete) {
      var b : Body = m_bodyList;
      while (b != null) {
        b.m_flags &= ~Body.e_islandFlag;
        b.m_sweep.alpha0 = 0.0;
        b = b.m_next;
      }

      var c : Contact = m_contactManager.m_contactList;
      while (c != null) {
        // Invalidate TOI
        c.m_flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
        c.m_toiCount = 0;
        c.m_toi = 1.0;
        c = c.m_next;
      }
    }

    // Find TOI events and solve them.
    while (true) {
      // Find the first TOI.
      var minContact : Contact = null;
      var minAlpha : Float = 1.0;

      var c : Contact = m_contactManager.m_contactList;
      while (c != null) {
        // Is this contact disabled?
        if (c.isEnabled() == false) {
          c = c.m_next;
          continue;
        }

        // Prevent excessive sub-stepping.
        if (c.m_toiCount > Settings.maxSubSteps) {
          c = c.m_next;
          continue;
        }

        var alpha : Float = 1.0;
        if ((c.m_flags & Contact.TOI_FLAG) != 0) {
          // This contact has a valid cached TOI.
          alpha = c.m_toi;
        } else {
          var fA : Fixture = c.getFixtureA();
          var fB : Fixture = c.getFixtureB();

          // Is there a sensor?
          if (fA.isSensor() || fB.isSensor()) {
            c = c.m_next;
            continue;
          }

          var bA : Body = fA.getBody();
          var bB : Body = fB.getBody();

          var typeA : BodyType = bA.m_type;
          var typeB : BodyType = bB.m_type;

          var activeA : Bool = bA.isAwake() && typeA != BodyType.STATIC;
          var activeB : Bool = bB.isAwake() && typeB != BodyType.STATIC;

          // Is at least one body active (awake and dynamic or kinematic)?
          if (activeA == false && activeB == false) {
            c = c.m_next;
            continue;
          }

          var collideA : Bool = bA.isBullet() || typeA != BodyType.DYNAMIC;
          var collideB : Bool = bB.isBullet() || typeB != BodyType.DYNAMIC;

          // Are these two non-bullet dynamic bodies?
          if (collideA == false && collideB == false) {
            c = c.m_next;
            continue;
          }

          // Compute the TOI for this contact.
          // Put the sweeps onto the same time interval.
          var alpha0 : Float = bA.m_sweep.alpha0;

          if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
            alpha0 = bB.m_sweep.alpha0;
            bA.m_sweep.advance(alpha0);
          } else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
            alpha0 = bA.m_sweep.alpha0;
            bB.m_sweep.advance(alpha0);
          }


          var indexA : Int = c.getChildIndexA();
          var indexB : Int = c.getChildIndexB();

          // Compute the time of impact in interval [0, minTOI]
          var input : TOIInput = toiInput;
          input.proxyA.set(fA.getShape(), indexA);
          input.proxyB.set(fB.getShape(), indexB);
          input.sweepA.set(bA.m_sweep);
          input.sweepB.set(bB.m_sweep);
          input.tMax = 1.0;

          pool.getTimeOfImpact().timeOfImpact(toiOutput, input);

          // Beta is the fraction of the remaining portion of the .
          var beta : Float = toiOutput.t;
          if (toiOutput.state == TOIOutputState.TOUCHING) {
            alpha = MathUtils.min(alpha0 + (1.0 - alpha0) * beta, 1.0);
          } else {
            alpha = 1.0;
          }

          c.m_toi = alpha;
          c.m_flags |= Contact.TOI_FLAG;
        }

        if (alpha < minAlpha) {
          // This is the minimum TOI found so far.
          minContact = c;
          minAlpha = alpha;
        }

        c = c.m_next;
      }

      if (minContact == null || 1.0 - 10.0 * Settings.EPSILON < minAlpha) {
        // No more TOI events. Done!
        m_stepComplete = true;
        break;
      }

      // Advance the bodies to the TOI.
      var fA : Fixture = minContact.getFixtureA();
      var fB : Fixture = minContact.getFixtureB();
      var bA : Body = fA.getBody();
      var bB : Body = fB.getBody();

      backup1.set(bA.m_sweep);
      backup2.set(bB.m_sweep);

      bA.advance(minAlpha);
      bB.advance(minAlpha);

      // The TOI contact likely has some new contact points.
      minContact.update(m_contactManager.m_contactListener);
      minContact.m_flags &= ~Contact.TOI_FLAG;
      ++minContact.m_toiCount;

      // Is the contact solid?
      if (minContact.isEnabled() == false || minContact.isTouching() == false) {
        // Restore the sweeps.
        minContact.setEnabled(false);
        bA.m_sweep.set(backup1);
        bB.m_sweep.set(backup2);
        bA.synchronizeTransform();
        bB.synchronizeTransform();
        continue;
      }

      bA.setAwake(true);
      bB.setAwake(true);

      // Build the island
      island.clear();
      island.addBody(bA);
      island.addBody(bB);
      island.addContact(minContact);

      bA.m_flags |= Body.e_islandFlag;
      bB.m_flags |= Body.e_islandFlag;
      minContact.m_flags |= Contact.ISLAND_FLAG;

      // Get contacts on bodyA and bodyB.
      tempBodies[0] = bA;
      tempBodies[1] = bB;
      for (i in 0 ... 2) {
      // for (int i = 0; i < 2; ++i) {
        var body : Body = tempBodies[i];
        if (body.m_type == BodyType.DYNAMIC) {
          var ce : ContactEdge = body.m_contactList;
          while (ce != null) {
            if (island.m_bodyCount == island.m_bodyCapacity) {
              break;
            }

            if (island.m_contactCount == island.m_contactCapacity) {
              break;
            }

            var contact : Contact = ce.contact;

            // Has this contact already been added to the island?
            if ((contact.m_flags & Contact.ISLAND_FLAG) != 0) {
              ce = ce.next;
              continue;
            }

            // Only add static, kinematic, or bullet bodies.
            var other : Body = ce.other;
            if (other.m_type == BodyType.DYNAMIC && body.isBullet() == false
                && other.isBullet() == false) {
              ce = ce.next;
              continue;
            }

            // Skip sensors.
            var sensorA : Bool = contact.m_fixtureA.m_isSensor;
            var sensorB : Bool = contact.m_fixtureB.m_isSensor;
            if (sensorA || sensorB) {
              ce = ce.next;
              continue;
            }

            // Tentatively advance the body to the TOI.
            backup1.set(other.m_sweep);
            if ((other.m_flags & Body.e_islandFlag) == 0) {
              other.advance(minAlpha);
            }

            // Update the contact points
            contact.update(m_contactManager.m_contactListener);

            // Was the contact disabled by the user?
            if (contact.isEnabled() == false) {
              other.m_sweep.set(backup1);
              other.synchronizeTransform();
              ce = ce.next;
              continue;
            }

            // Are there contact points?
            if (contact.isTouching() == false) {
              other.m_sweep.set(backup1);
              other.synchronizeTransform();
              ce = ce.next;
              continue;
            }

            // Add the contact to the island
            contact.m_flags |= Contact.ISLAND_FLAG;
            island.addContact(contact);

            // Has the other body already been added to the island?
            if ((other.m_flags & Body.e_islandFlag) != 0) {
              ce = ce.next;
              continue;
            }

            // Add the other body to the island.
            other.m_flags |= Body.e_islandFlag;

            if (other.m_type != BodyType.STATIC) {
              other.setAwake(true);
            }

            island.addBody(other);

            ce = ce.next;
          }
        }
      }

      subStep.dt = (1.0 - minAlpha) * step.dt;
      subStep.inv_dt = 1.0 / subStep.dt;
      subStep.dtRatio = 1.0;
      subStep.positionIterations = 20;
      subStep.velocityIterations = step.velocityIterations;
      subStep.warmStarting = false;
      island.solveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

      // Reset island flags and synchronize broad-phase proxies.
      for (i in 0 ... island.m_bodyCount) {
      // for (int i = 0; i < island.m_bodyCount; ++i) {
        var body : Body = island.m_bodies[i];
        body.m_flags &= ~Body.e_islandFlag;

        if (body.m_type != BodyType.DYNAMIC) {
          continue;
        }

        body.synchronizeFixtures();

        // Invalidate all contact TOIs on this displaced body.
        var ce : ContactEdge = body.m_contactList;
        while (ce != null) {
          ce.contact.m_flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
          ce = ce.next;
        }
      }

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      m_contactManager.findNewContacts();

      if (m_subStepping) {
        m_stepComplete = false;
        break;
      }
    }
  }

  private function drawJoint(joint : Joint) : Void {
    var bodyA : Body = joint.getBodyA();
    var bodyB : Body = joint.getBodyB();
    var xf1 : Transform = bodyA.getTransform();
    var xf2 : Transform = bodyB.getTransform();
    var x1 : Vec2 = xf1.p;
    var x2 : Vec2 = xf2.p;
    var p1 : Vec2 = pool.popVec2();
    var p2 : Vec2 = pool.popVec2();
    joint.getAnchorA(p1);
    joint.getAnchorB(p2);

    color.set(0.5, 0.8, 0.8);

    switch (joint.getType()) {
    // TODO djm write after writing joints
      case DISTANCE:
        m_debugDraw.drawSegment(p1, p2, color);

      case PULLEY: {
        var pulley : PulleyJoint = cast joint;
        var s1 : Vec2 = pulley.getGroundAnchorA();
        var s2 : Vec2 = pulley.getGroundAnchorB();
        m_debugDraw.drawSegment(s1, p1, color);
        m_debugDraw.drawSegment(s2, p2, color);
        m_debugDraw.drawSegment(s1, s2, color);
      }
       
      case CONSTANT_VOLUME:
      case MOUSE:
        // don't draw this
       
      default:
        m_debugDraw.drawSegment(x1, p1, color);
        m_debugDraw.drawSegment(p1, p2, color);
        m_debugDraw.drawSegment(x2, p2, color);
    }
    pool.pushVec2(2);
  }

  // NOTE this corresponds to the liquid test, so the debugdraw can draw
  // the liquid particles correctly. They should be the same.
  // TODO: Integer cannot be constructed
  // private static var LIQUID_INT : Int = new Int(1234598372);
  private static var LIQUID_INT : Int = 1234598372;
  private var liquidLength : Float = .12;
  private var averageLinearVel : Float = -1;
  private var liquidOffset : Vec2 = new Vec2();
  private var circCenterMoved : Vec2 = new Vec2();
  private var liquidColor : Color3f = new Color3f(.4, .4, 1);

  private var center : Vec2 = new Vec2();
  private var axis : Vec2 = new Vec2();
  private var v1 : Vec2 = new Vec2();
  private var v2 : Vec2 = new Vec2();
  private var tlvertices : Vec2Array = new Vec2Array();

  private function drawShape(fixture : Fixture, xf : Transform, color : Color3f, wireframe : Bool) : Void {
    switch (fixture.getType()) {
      case CIRCLE: {
        var circle : CircleShape = cast fixture.getShape();

        // Vec2 center = Mul(xf, circle.m_p);
        Transform.mulToOutUnsafe(xf, circle.m_p, center);
        var radius : Float = circle.m_radius;
        xf.q.getXAxis(axis);

        if (fixture.getUserData() != null && fixture.getUserData().equals(LIQUID_INT)) {
          var b : Body = fixture.getBody();
          liquidOffset.setVec(b.m_linearVelocity);
          var linVelLength : Float = b.m_linearVelocity.length();
          if (averageLinearVel == -1) {
            averageLinearVel = linVelLength;
          } else {
            averageLinearVel = .98 * averageLinearVel + .02 * linVelLength;
          }
          liquidOffset.mulLocal(liquidLength / averageLinearVel / 2);
          circCenterMoved.setVec(center).addLocalVec(liquidOffset);
          center.subLocal(liquidOffset);
          m_debugDraw.drawSegment(center, circCenterMoved, liquidColor);
          return;
        }
        if (wireframe) {
          m_debugDraw.drawCircle2(center, radius, axis, color);
        } else {
          m_debugDraw.drawSolidCircle(center, radius, axis, color);
        }
      }

      case POLYGON: {
        var poly : PolygonShape = cast fixture.getShape();
        var vertexCount : Int = poly.m_count;
        var vertices : Vector<Vec2> = tlvertices.get(Settings.maxPolygonVertices);

        for (i in 0 ... vertexCount) {
        // for (int i = 0; i < vertexCount; ++i) {
          // vertices[i] = Mul(xf, poly.m_vertices[i]);
          Transform.mulToOutUnsafe(xf, poly.m_vertices[i], vertices[i]);
        }
        if (wireframe) {
          m_debugDraw.drawPolygon(vertices, vertexCount, color);
        } else {
          m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);
        }
      }

      case EDGE: {
        var edge : EdgeShape = cast fixture.getShape();
        Transform.mulToOutUnsafe(xf, edge.m_vertex1, v1);
        Transform.mulToOutUnsafe(xf, edge.m_vertex2, v2);
        m_debugDraw.drawSegment(v1, v2, color);
      }
        
      case CHAIN: {
        var chain : ChainShape = cast fixture.getShape();
        var count : Int = chain.m_count;
        var vertices : Vector<Vec2> = chain.m_vertices;

        Transform.mulToOutUnsafe(xf, vertices[0], v1);
        for (i in 1 ... count) {
        // for (int i = 1; i < count; ++i) {
          Transform.mulToOutUnsafe(xf, vertices[i], v2);
          m_debugDraw.drawSegment(v1, v2, color);
          m_debugDraw.drawCircle(v1, 0.05, color);
          v1.setVec(v2);
        }
      }
        
      default:
    }
  }

  private function drawParticleSystem(system : ParticleSystem) : Void {
    var wireframe : Bool = (m_debugDraw.getFlags() & DebugDraw.e_wireframeDrawingBit) != 0;
    var particleCount : Int = system.getParticleCount();
    if (particleCount != 0) {
      var particleRadius : Float = system.getParticleRadius();
      var positionBuffer : Vector<Vec2> = system.getParticlePositionBuffer();
      var colorBuffer : Vector<ParticleColor> = null;
      if (system.m_colorBuffer.data != null) {
        colorBuffer = system.getParticleColorBuffer();
      }
      if (wireframe) {
        m_debugDraw.drawParticlesWireframe(positionBuffer, particleRadius, colorBuffer,
            particleCount);
      } else {
        m_debugDraw.drawParticles(positionBuffer, particleRadius, colorBuffer, particleCount);
      }
    }
  }

  /**
   * Create a particle whose properties have been defined. No reference to the definition is
   * retained. A simulation step must occur before it's possible to interact with a newly created
   * particle. For example, DestroyParticleInShape() will not destroy a particle until Step() has
   * been called.
   * 
   * @warning This function is locked during callbacks.
   * @return the index of the particle.
   */
  public function createParticle(def : ParticleDef) : Int {
    if (isLocked()) {
      return 0;
    }
    var p : Int = m_particleSystem.createParticle(def);
    return p;
  }

  /**
   * Destroy a particle. The particle is removed after the next step.
   * 
   * @param index
   */
  public function destroyParticle(index : Int) : Void {
    destroyParticle2(index, false);
  }

  /**
   * Destroy a particle. The particle is removed after the next step.
   * 
   * @param Index of the particle to destroy.
   * @param Whether to call the destruction listener just before the particle is destroyed.
   */
  public function destroyParticle2(index : Int, callDestructionListener : Bool) : Void {
    m_particleSystem.destroyParticle(index, callDestructionListener);
  }

  /**
   * Destroy particles inside a shape without enabling the destruction callback for destroyed
   * particles. This function is locked during callbacks. For more information see
   * DestroyParticleInShape(Shape&, Transform&,bool).
   * 
   * @param Shape which encloses particles that should be destroyed.
   * @param Transform applied to the shape.
   * @warning This function is locked during callbacks.
   * @return Number of particles destroyed.
   */
  public function destroyParticlesInShape(shape : Shape, xf : Transform) : Int {
    return destroyParticlesInShape2(shape, xf, false);
  }

  /**
   * Destroy particles inside a shape. This function is locked during callbacks. In addition, this
   * function immediately destroys particles in the shape in contrast to DestroyParticle() which
   * defers the destruction until the next simulation step.
   * 
   * @param Shape which encloses particles that should be destroyed.
   * @param Transform applied to the shape.
   * @param Whether to call the world b2DestructionListener for each particle destroyed.
   * @warning This function is locked during callbacks.
   * @return Number of particles destroyed.
   */
  public function destroyParticlesInShape2(shape : Shape, xf : Transform, callDestructionListener : Bool) : Int {
    if (isLocked()) {
      return 0;
    }
    return m_particleSystem.destroyParticlesInShape(shape, xf, callDestructionListener);
  }

  /**
   * Create a particle group whose properties have been defined. No reference to the definition is
   * retained.
   * 
   * @warning This function is locked during callbacks.
   */
  public function createParticleGroup(def : ParticleGroupDef) : ParticleGroup {
    if (isLocked()) {
      return null;
    }
    var g : ParticleGroup = m_particleSystem.createParticleGroup(def);
    return g;
  }

  /**
   * Join two particle groups.
   * 
   * @param the first group. Expands to encompass the second group.
   * @param the second group. It is destroyed.
   * @warning This function is locked during callbacks.
   */
  public function joinParticleGroups(groupA : ParticleGroup, groupB : ParticleGroup) : Void {
    if (isLocked()) {
      return;
    }
    m_particleSystem.joinParticleGroups(groupA, groupB);
  }

  /**
   * Destroy particles in a group. This function is locked during callbacks.
   * 
   * @param The particle group to destroy.
   * @param Whether to call the world b2DestructionListener for each particle is destroyed.
   * @warning This function is locked during callbacks.
   */
  public function destroyParticlesInGroup(group : ParticleGroup, callDestructionListener : Bool) : Void {
    if (isLocked()) {
      return;
    }
    m_particleSystem.destroyParticlesInGroup(group, callDestructionListener);
  }

  /**
   * Destroy particles in a group without enabling the destruction callback for destroyed particles.
   * This function is locked during callbacks.
   * 
   * @param The particle group to destroy.
   * @warning This function is locked during callbacks.
   */
  public function destroyParticlesInGroup2(group : ParticleGroup) : Void {
    destroyParticlesInGroup(group, false);
  }

  /**
   * Get the world particle group list. With the returned group, use ParticleGroup::GetNext to get
   * the next group in the world list. A NULL group indicates the end of the list.
   * 
   * @return the head of the world particle group list.
   */
  public function getParticleGroupList() : Vector<ParticleGroup> {
    return m_particleSystem.getParticleGroupList();
  }

  /**
   * Get the number of particle groups.
   * 
   * @return
   */
  public function getParticleGroupCount() : Int {
    return m_particleSystem.getParticleGroupCount();
  }

  /**
   * Get the number of particles.
   * 
   * @return
   */
  public function getParticleCount() : Int {
    return m_particleSystem.getParticleCount();
  }

  /**
   * Get the maximum number of particles.
   * 
   * @return
   */
  public function getParticleMaxCount() : Int {
    return m_particleSystem.getParticleMaxCount();
  }

  /**
   * Set the maximum number of particles.
   * 
   * @param count
   */
  public function setParticleMaxCount(count : Int) : Void {
    m_particleSystem.setParticleMaxCount(count);
  }

  /**
   * Change the particle density.
   * 
   * @param density
   */
  public function setParticleDensity(density : Float) : Void {
    m_particleSystem.setParticleDensity(density);
  }

  /**
   * Get the particle density.
   * 
   * @return
   */
  public function getParticleDensity() : Float {
    return m_particleSystem.getParticleDensity();
  }

  /**
   * Change the particle gravity scale. Adjusts the effect of the global gravity vector on
   * particles. Default value is 1.0f.
   * 
   * @param gravityScale
   */
  public function setParticleGravityScale(gravityScale : Float) : Void {
    m_particleSystem.setParticleGravityScale(gravityScale);

  }

  /**
   * Get the particle gravity scale.
   * 
   * @return
   */
  public function getParticleGravityScale() : Float {
    return m_particleSystem.getParticleGravityScale();
  }

  /**
   * Damping is used to reduce the velocity of particles. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   * 
   * @param damping
   */
  public function setParticleDamping(damping : Float) : Void {
    m_particleSystem.setParticleDamping(damping);
  }

  /**
   * Get damping for particles
   * 
   * @return
   */
  public function getParticleDamping() : Float {
    return m_particleSystem.getParticleDamping();
  }

  /**
   * Change the particle radius. You should set this only once, on world start. If you change the
   * radius during execution, existing particles may explode, shrink, or behave unexpectedly.
   * 
   * @param radius
   */
  public function setParticleRadius(radius : Float) : Void {
    m_particleSystem.setParticleRadius(radius);
  }

  /**
   * Get the particle radius.
   * 
   * @return
   */
  public function getParticleRadius() : Float {
    return m_particleSystem.getParticleRadius();
  }

  /**
   * Get the particle data. @return the pointer to the head of the particle data.
   * 
   * @return
   */
  public function getParticleFlagsBuffer() : Vector<Int> {
    return m_particleSystem.getParticleFlagsBuffer();
  }

  public function getParticlePositionBuffer() : Vector<Vec2> {
    return m_particleSystem.getParticlePositionBuffer();
  }

  public function getParticleVelocityBuffer() : Vector<Vec2> {
    return m_particleSystem.getParticleVelocityBuffer();
  }

  public function getParticleColorBuffer() : Vector<ParticleColor> {
    return m_particleSystem.getParticleColorBuffer();
  }

  public function getParticleGroupBuffer() : Vector<ParticleGroup> {
    return m_particleSystem.getParticleGroupBuffer();
  }

  public function getParticleUserDataBuffer() : Vector<Dynamic> {
    return m_particleSystem.getParticleUserDataBuffer();
  }

  /**
   * Set a buffer for particle data.
   * 
   * @param buffer is a pointer to a block of memory.
   * @param size is the number of values in the block.
   */
  public function setParticleFlagsBuffer(buffer : Vector<Int>, capacity : Int) : Void {
    m_particleSystem.setParticleFlagsBuffer(buffer, capacity);
  }

  public function setParticlePositionBuffer(buffer : Vector<Vec2>, capacity : Int) : Void {
    m_particleSystem.setParticlePositionBuffer(buffer, capacity);

  }

  public function setParticleVelocityBuffer(buffer : Vector<Vec2>, capacity : Int) : Void {
    m_particleSystem.setParticleVelocityBuffer(buffer, capacity);

  }

  public function setParticleColorBuffer(buffer : Vector<ParticleColor>, capacity : Int) : Void {
    m_particleSystem.setParticleColorBuffer(buffer, capacity);

  }

  public function setParticleUserDataBuffer(buffer : Dynamic, capacity : Int) : Void {
    m_particleSystem.setParticleUserDataBuffer(buffer, capacity);
  }

  /**
   * Get contacts between particles
   * 
   * @return
   */
  public function getParticleContacts() : Vector<ParticleContact> {
    return m_particleSystem.m_contactBuffer;
  }

  public function getParticleContactCount() : Int {
    return m_particleSystem.m_contactCount;
  }

  /**
   * Get contacts between particles and bodies
   * 
   * @return
   */
  public function getParticleBodyContacts() : Vector<ParticleBodyContact> {
    return m_particleSystem.m_bodyContactBuffer;
  }

  public function getParticleBodyContactCount() : Int {
    return m_particleSystem.m_bodyContactCount;
  }

  /**
   * Compute the kinetic energy that can be lost by damping force
   * 
   * @return
   */
  public function computeParticleCollisionEnergy() : Float {
    return m_particleSystem.computeParticleCollisionEnergy();
  }
}


class WorldQueryWrapper implements TreeCallback {
  public var broadPhase : BroadPhase;
  public var callback : QueryCallback;

  public function new() {}

  public function treeCallback(nodeId : Int) : Bool {
    var proxy : FixtureProxy = cast broadPhase.getUserData(nodeId);
    return callback.reportFixture(proxy.fixture);
  }

}


class WorldRayCastWrapper implements TreeRayCastCallback {

  public var broadPhase : BroadPhase;
  public var callback : RayCastCallback;

  // djm pooling
  private var output : RayCastOutput = new RayCastOutput();
  private var temp : Vec2 = new Vec2();
  private var point : Vec2 = new Vec2();

  public function new() {}

  public function raycastCallback(input : RayCastInput, nodeId : Int) : Float {
    var userData : Dynamic = broadPhase.getUserData(nodeId);
    var proxy : FixtureProxy = cast userData;
    var fixture : Fixture = proxy.fixture;
    var index : Int = proxy.childIndex;
    var hit : Bool = fixture.raycast(output, input, index);

    if (hit) {
      var fraction : Float = output.fraction;
      // Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
      temp.setVec(input.p2).mulLocal(fraction);
      point.setVec(input.p1).mulLocal(1 - fraction).addLocalVec(temp);
      return callback.reportFixture(fixture, point, output.normal, fraction);
    }

    return input.maxFraction;
  }

}