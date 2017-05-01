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

import box2d.callbacks.ContactImpulse;
import box2d.callbacks.ContactListener;
import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Sweep;
import box2d.common.TimerB2d;
import box2d.common.Vec2;
import box2d.dynamics.contacts.Contact;
import box2d.dynamics.contacts.ContactSolver;
import box2d.dynamics.contacts.ContactSolverDef;
import box2d.dynamics.contacts.ContactVelocityConstraint;
import box2d.dynamics.contacts.Position;
import box2d.dynamics.contacts.Velocity;
import box2d.dynamics.joints.Joint;
import haxe.ds.Vector;

/*
 Position Correction Notes
 =========================
 I tried the several algorithms for position correction of the 2D revolute joint.
 I looked at these systems:
 - simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
 - suspension bridge with 30 1m long planks of length 1m.
 - multi-link chain with 30 1m long links.

 Here are the algorithms:

 Baumgarte - A fraction of the position error is added to the velocity error. There is no
 separate position solver.

 Pseudo Velocities - After the velocity solver and position integration,
 the position error, Jacobian, and effective mass are recomputed. Then
 the velocity constraints are solved with pseudo velocities and a fraction
 of the position error is added to the pseudo velocity error. The pseudo
 velocities are initialized to zero and there is no warm-starting. After
 the position solver, the pseudo velocities are added to the positions.
 This is also called the First Order World method or the Position LCP method.

 Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
 position error is re-computed for each raint and the positions are updated
 after the raint is solved. The radius vectors (aka Jacobians) are
 re-computed too (otherwise the algorithm has horrible instability). The pseudo
 velocity states are not needed because they are effectively zero at the beginning
 of each iteration. Since we have the current position error, we allow the
 iterations to terminate early if the error becomes smaller than Settings.linearSlop.

 Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
 each time a raint is solved.

 Here are the results:
 Baumgarte - this is the cheapest algorithm but it has some stability problems,
 especially with the bridge. The chain links separate easily close to the root
 and they jitter as they struggle to pull together. This is one of the most common
 methods in the field. The big drawback is that the position correction artificially
 affects the momentum, thus leading to instabilities and false bounce. I used a
 bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
 factor makes joints and contacts more spongy.

 Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
 stable. However, joints still separate with large angular velocities. Drag the
 simple pendulum in a circle quickly and the joint will separate. The chain separates
 easily and does not recover. I used a bias factor of 0.2. A larger value lead to
 the bridge collapsing when a heavy cube drops on it.

 Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
 Velocities, but in other ways it is worse. The bridge and chain are much more
 stable, but the simple pendulum goes unstable at high angular velocities.

 Full NGS - stable in all tests. The joints display good stiffness. The bridge
 still sags, but this is better than infinite forces.

 Recommendations
 Pseudo Velocities are not really worthwhile because the bridge and chain cannot
 recover from joint separation. In other cases the benefit over Baumgarte is small.

 Modified NGS is not a robust method for the revolute joint due to the violent
 instability seen in the simple pendulum. Perhaps it is viable with other raint
 types, especially scalar constraints where the effective mass is a scalar.

 This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
 and is very fast. I don't think we can escape Baumgarte, especially in highly
 demanding cases where high raint fidelity is not needed.

 Full NGS is robust and easy on the eyes. I recommend this as an option for
 higher fidelity simulation and certainly for suspension bridges and long chains.
 Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
 joint separation can be problematic. The number of NGS iterations can be reduced
 for better performance without harming robustness much.

 Each joint in a can be handled differently in the position solver. So I recommend
 a system where the user can select the algorithm on a per joint basis. I would
 probably default to the slower Full NGS and let the user select the faster
 Baumgarte method in performance critical scenarios.
 */

/*
 Cache Performance

 The Box2D solvers are dominated by cache misses. Data structures are designed
 to increase the number of cache hits. Much of misses are due to random access
 to body data. The raint structures are iterated over linearly, which leads
 to few cache misses.

 The bodies are not accessed during iteration. Instead read only data, such as
 the mass values are stored with the constraints. The mutable data are the raint
 impulses and the bodies velocities/positions. The impulses are held inside the
 raint structures. The body velocities/positions are held in compact, temporary
 arrays to increase the number of cache hits. Linear and angular velocity are
 stored in a single array since multiple arrays lead to multiple misses.
 */

/*
 2D Rotation

 R = [cos(theta) -sin(theta)]
 [sin(theta) cos(theta) ]

 thetaDot = omega

 Let q1 = cos(theta), q2 = sin(theta).
 R = [q1 -q2]
 [q2  q1]

 q1Dot = -thetaDot * q2
 q2Dot = thetaDot * q1

 q1_new = q1_old - dt * w * q2
 q2_new = q2_old + dt * w * q1
 then normalize.

 This might be faster than computing sin+cos.
 However, we can compute sin+cos of the same angle fast.
 */

/**
 * This is an internal class.
 * 
 * @author Daniel Murphy
 */
 class Island {

  public var m_listener : ContactListener;

  public var m_bodies : Vector<Body>;
  public var m_contacts : Vector<Contact>;
  public var m_joints : Vector<Joint>;

  public var m_positions : Vector<Position>;
  public var m_velocities : Vector<Velocity>;

  public var m_bodyCount : Int;
  public var m_jointCount : Int;
  public var m_contactCount : Int;

  public var m_bodyCapacity : Int;
  public var m_contactCapacity : Int;
  public var m_jointCapacity : Int;

  public function new() {

  }

  public function init(bodyCapacity : Int, contactCapacity : Int, jointCapacity : Int,
      listener : ContactListener) : Void {
    // System.out.println("Initializing Island");
    m_bodyCapacity = bodyCapacity;
    m_contactCapacity = contactCapacity;
    m_jointCapacity = jointCapacity;
    m_bodyCount = 0;
    m_contactCount = 0;
    m_jointCount = 0;

    m_listener = listener;

    if (m_bodies == null || m_bodyCapacity > m_bodies.length) {
      m_bodies = new Vector<Body>(m_bodyCapacity);
    }
    if (m_joints == null || m_jointCapacity > m_joints.length) {
      m_joints = new Vector<Joint>(m_jointCapacity);
    }
    if (m_contacts == null || m_contactCapacity > m_contacts.length) {
      m_contacts = new Vector<Contact>(m_contactCapacity);
    }

    // dynamic array
    if (m_velocities == null || m_bodyCapacity > m_velocities.length) {
      var old : Vector<Velocity> = m_velocities == null ? new Vector<Velocity>(0) : m_velocities;
      m_velocities = new Vector<Velocity>(m_bodyCapacity);
      // TODO: array copy
      Vector.blit(old, 0, m_velocities, 0, old.length);
      // System.arraycopy(old, 0, m_velocities, 0, old.length);
      for(i in old.length ... m_velocities.length) {
        m_velocities[i] = new Velocity();
      }
    }

    // dynamic array
    if (m_positions == null || m_bodyCapacity > m_positions.length) {
      var old : Vector<Position> = m_positions == null ? new Vector<Position>(0) : m_positions;
      m_positions = new Vector<Position>(m_bodyCapacity);
      // TODO: array copy
      Vector.blit(old, 0, m_positions, 0, old.length);
      // System.arraycopy(old, 0, m_positions, 0, old.length);
      for(i in old.length ... m_positions.length) {
        m_positions[i] = new Position();
      }
    }
  }

  public function clear() : Void {
    m_bodyCount = 0;
    m_contactCount = 0;
    m_jointCount = 0;
  }

  private var contactSolver : ContactSolver = new ContactSolver();
  private var timer : TimerB2d = new TimerB2d();
  private var solverData : SolverData = new SolverData();
  private var solverDef : ContactSolverDef = new ContactSolverDef();

  public function solve(profile : Profile, step : TimeStep, gravity : Vec2, allowSleep : Bool) : Void {

    // System.out.println("Solving Island");
    var h : Float = step.dt;

    // Integrate velocities and apply damping. Initialize the body state.
    for (i in 0 ... m_bodyCount) {
    // for (int i = 0; i < m_bodyCount; ++i) {
      var b : Body = m_bodies[i];
      var bm_sweep : Sweep = b.m_sweep;
      var c : Vec2 = bm_sweep.c;
      var a : Float = bm_sweep.a;
      var v : Vec2 = b.m_linearVelocity;
      var w : Float = b.m_angularVelocity;

      // Store positions for continuous collision.
      bm_sweep.c0.setVec(bm_sweep.c);
      bm_sweep.a0 = bm_sweep.a;

      if (b.m_type == BodyType.DYNAMIC) {
        // Integrate velocities.
        // v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
        v.x += h * (b.m_gravityScale * gravity.x + b.m_invMass * b.m_force.x);
        v.y += h * (b.m_gravityScale * gravity.y + b.m_invMass * b.m_force.y);
        w += h * b.m_invI * b.m_torque;

        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
        // exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        v.x *= 1.0 / (1.0 + h * b.m_linearDamping);
        v.y *= 1.0 / (1.0 + h * b.m_linearDamping);
        w *= 1.0 / (1.0 + h * b.m_angularDamping);
      }

      m_positions[i].c.x = c.x;
      m_positions[i].c.y = c.y;
      m_positions[i].a = a;
      m_velocities[i].v.x = v.x;
      m_velocities[i].v.y = v.y;
      m_velocities[i].w = w;
    }

    timer.reset();

    // Solver data
    solverData.step = step;
    solverData.positions = m_positions;
    solverData.velocities = m_velocities;

    // Initialize velocity constraints.
    solverDef.step = step;
    solverDef.contacts = m_contacts;
    solverDef.count = m_contactCount;
    solverDef.positions = m_positions;
    solverDef.velocities = m_velocities;

    contactSolver.init(solverDef);
    // System.out.println("island init vel");
    contactSolver.initializeVelocityConstraints();

    if (step.warmStarting) {
      // System.out.println("island warm start");
      contactSolver.warmStart();
    }

    for (i in 0 ... m_jointCount) {
    // for (int i = 0; i < m_jointCount; ++i) {
      m_joints[i].initVelocityConstraints(solverData);
    }

    profile.solveInit.accumValue(timer.getMilliseconds());

    // Solve velocity constraints
    timer.reset();
    // System.out.println("island solving velocities");
    for (i in 0 ... step.velocityIterations) {
    // for (int i = 0; i < step.velocityIterations; ++i) {
      for (j in 0 ... m_jointCount) {
      // for (int j = 0; j < m_jointCount; ++j) {
        m_joints[j].solveVelocityConstraints(solverData);
      }

      contactSolver.solveVelocityConstraints();
    }

    // Store impulses for warm starting
    contactSolver.storeImpulses();
    profile.solveVelocity.accumValue(timer.getMilliseconds());

    // Integrate positions
    for (i in 0 ... m_bodyCount) {
    // for (int i = 0; i < m_bodyCount; ++i) {
      var c : Vec2 = m_positions[i].c;
      var a : Float = m_positions[i].a;
      var v : Vec2 = m_velocities[i].v;
      var w : Float = m_velocities[i].w;

      // Check for large velocities
      var translationx : Float = v.x * h;
      var translationy : Float = v.y * h;

      if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
        var ratio : Float = Settings.maxTranslation
            / MathUtils.sqrt(translationx * translationx + translationy * translationy);
        v.x *= ratio;
        v.y *= ratio;
      }

      var rotation : Float = h * w;
      if (rotation * rotation > Settings.maxRotationSquared) {
        var ratio : Float = Settings.maxRotation / MathUtils.abs(rotation);
        w *= ratio;
      }

      // Integrate
      c.x += h * v.x;
      c.y += h * v.y;
      a += h * w;

      m_positions[i].a = a;
      m_velocities[i].w = w;
    }

    // Solve position constraints
    timer.reset();
    var positionSolved : Bool = false;
    for (i in 0 ... step.positionIterations) {
    // for (int i = 0; i < step.positionIterations; ++i) {
      var contactsOkay : Bool = contactSolver.solvePositionConstraints();

      var jointsOkay : Bool = true;
      for (j in 0 ... m_jointCount) {
      // for (int j = 0; j < m_jointCount; ++j) {
        var jointOkay : Bool = m_joints[j].solvePositionConstraints(solverData);
        jointsOkay = jointsOkay && jointOkay;
      }

      if (contactsOkay && jointsOkay) {
        // Exit early if the position errors are small.
        positionSolved = true;
        break;
      }
    }

    // Copy state buffers back to the bodies
    for (i in 0 ... m_bodyCount) {
    // for (int i = 0; i < m_bodyCount; ++i) {
      var body : Body = m_bodies[i];
      body.m_sweep.c.x = m_positions[i].c.x;
      body.m_sweep.c.y = m_positions[i].c.y;
      body.m_sweep.a = m_positions[i].a;
      body.m_linearVelocity.x = m_velocities[i].v.x;
      body.m_linearVelocity.y = m_velocities[i].v.y;
      body.m_angularVelocity = m_velocities[i].w;
      body.synchronizeTransform();
    }

    profile.solvePosition.accumValue(timer.getMilliseconds());

    report(contactSolver.m_velocityConstraints);

    if (allowSleep) {
      var minSleepTime : Float = MathUtils.MAX_VALUE;

      var linTolSqr : Float = Settings.linearSleepTolerance * Settings.linearSleepTolerance;
      var angTolSqr : Float = Settings.angularSleepTolerance * Settings.angularSleepTolerance;

      for (i in 0 ... m_bodyCount) {
      // for (int i = 0; i < m_bodyCount; ++i) {
        var b : Body = m_bodies[i];
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        if ((b.m_flags & Body.e_autoSleepFlag) == 0
            || b.m_angularVelocity * b.m_angularVelocity > angTolSqr
            || Vec2.dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
          b.m_sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b.m_sleepTime += h;
          minSleepTime = MathUtils.min(minSleepTime, b.m_sleepTime);
        }
      }

      if (minSleepTime >= Settings.timeToSleep && positionSolved) {
        for (i in 0 ... m_bodyCount) {
        // for (int i = 0; i < m_bodyCount; ++i) {
          var b : Body = m_bodies[i];
          b.setAwake(false);
        }
      }
    }
  }

  private var toiContactSolver : ContactSolver = new ContactSolver();
  private var toiSolverDef : ContactSolverDef = new ContactSolverDef();

  public function solveTOI(subStep : TimeStep, toiIndexA : Int, toiIndexB : Int) : Void {

    // Initialize the body state.
    for (i in 0 ... m_bodyCount) {
    // for (int i = 0; i < m_bodyCount; ++i) {
      m_positions[i].c.x = m_bodies[i].m_sweep.c.x;
      m_positions[i].c.y = m_bodies[i].m_sweep.c.y;
      m_positions[i].a = m_bodies[i].m_sweep.a;
      m_velocities[i].v.x = m_bodies[i].m_linearVelocity.x;
      m_velocities[i].v.y = m_bodies[i].m_linearVelocity.y;
      m_velocities[i].w = m_bodies[i].m_angularVelocity;
    }

    toiSolverDef.contacts = m_contacts;
    toiSolverDef.count = m_contactCount;
    toiSolverDef.step = subStep;
    toiSolverDef.positions = m_positions;
    toiSolverDef.velocities = m_velocities;
    toiContactSolver.init(toiSolverDef);

    // Solve position constraints.
    for (i in 0 ... subStep.positionIterations) {
    // for (int i = 0; i < subStep.positionIterations; ++i) {
      var contactsOkay : Bool = toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
      if (contactsOkay) {
        break;
      }
    }
    // #if 0
    // // Is the new position really safe?
    // for (int i = 0; i < m_contactCount; ++i)
    // {
    // Contact* c = m_contacts[i];
    // Fixture* fA = c.GetFixtureA();
    // Fixture* fB = c.GetFixtureB();
    //
    // Body bA = fA.GetBody();
    // Body bB = fB.GetBody();
    //
    // int indexA = c.GetChildIndexA();
    // int indexB = c.GetChildIndexB();
    //
    // DistanceInput input;
    // input.proxyA.Set(fA.GetShape(), indexA);
    // input.proxyB.Set(fB.GetShape(), indexB);
    // input.transformA = bA.GetTransform();
    // input.transformB = bB.GetTransform();
    // input.useRadii = false;
    //
    // DistanceOutput output;
    // SimplexCache cache;
    // cache.count = 0;
    // Distance(&output, &cache, &input);
    //
    // if (output.distance == 0 || cache.count == 3)
    // {
    // cache.count += 0;
    // }
    // }
    // #endif

    // Leap of faith to new safe state.
    m_bodies[toiIndexA].m_sweep.c0.x = m_positions[toiIndexA].c.x;
    m_bodies[toiIndexA].m_sweep.c0.y = m_positions[toiIndexA].c.y;
    m_bodies[toiIndexA].m_sweep.a0 = m_positions[toiIndexA].a;
    m_bodies[toiIndexB].m_sweep.c0.setVec(m_positions[toiIndexB].c);
    m_bodies[toiIndexB].m_sweep.a0 = m_positions[toiIndexB].a;

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.
    toiContactSolver.initializeVelocityConstraints();

    // Solve velocity constraints.
    for (i in 0 ... subStep.velocityIterations) {
    // for (int i = 0; i < subStep.velocityIterations; ++i) {
      toiContactSolver.solveVelocityConstraints();
    }

    // Don't store the TOI contact forces for warm starting
    // because they can be quite large.

    var h : Float = subStep.dt;

    // Integrate positions
    for (i in 0 ... m_bodyCount) {
    // for (int i = 0; i < m_bodyCount; ++i) {
      var c : Vec2 = m_positions[i].c;
      var a : Float = m_positions[i].a;
      var v : Vec2 = m_velocities[i].v;
      var w : Float = m_velocities[i].w;

      // Check for large velocities
      var translationx : Float = v.x * h;
      var translationy : Float = v.y * h;
      if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
        var ratio : Float =
            Settings.maxTranslation
                / MathUtils.sqrt(translationx * translationx + translationy * translationy);
        v.mulLocal(ratio);
      }

      var rotation : Float = h * w;
      if (rotation * rotation > Settings.maxRotationSquared) {
        var ratio : Float = Settings.maxRotation / MathUtils.abs(rotation);
        w *= ratio;
      }

      // Integrate
      c.x += v.x * h;
      c.y += v.y * h;
      a += h * w;

      m_positions[i].c.x = c.x;
      m_positions[i].c.y = c.y;
      m_positions[i].a = a;
      m_velocities[i].v.x = v.x;
      m_velocities[i].v.y = v.y;
      m_velocities[i].w = w;

      // Sync bodies
      var body : Body = m_bodies[i];
      body.m_sweep.c.x = c.x;
      body.m_sweep.c.y = c.y;
      body.m_sweep.a = a;
      body.m_linearVelocity.x = v.x;
      body.m_linearVelocity.y = v.y;
      body.m_angularVelocity = w;
      body.synchronizeTransform();
    }

    report(toiContactSolver.m_velocityConstraints);
  }

  public function addBody(body : Body) : Void {
    body.m_islandIndex = m_bodyCount;
    m_bodies[m_bodyCount] = body;
    ++m_bodyCount;
  }

  public function addContact(contact : Contact) : Void {
    m_contacts[m_contactCount++] = contact;
  }

  public function addJoint(joint : Joint) : Void {
    m_joints[m_jointCount++] = joint;
  }

  private var impulse : ContactImpulse = new ContactImpulse();

  public function report(constraints : Vector<ContactVelocityConstraint>) : Void {
    if (m_listener == null) {
      return;
    }

    for (i in 0 ... m_contactCount) {
    // for (int i = 0; i < m_contactCount; ++i) {
      var c : Contact = m_contacts[i];

      var vc : ContactVelocityConstraint = constraints[i];
      impulse.count = vc.pointCount;
      for (j in 0 ... vc.pointCount) {
      // for (int j = 0; j < vc.pointCount; ++j) {
        impulse.normalImpulses[j] = vc.points[j].normalImpulse;
        impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
      }

      m_listener.postSolve(c, impulse);
    }
  }
}

