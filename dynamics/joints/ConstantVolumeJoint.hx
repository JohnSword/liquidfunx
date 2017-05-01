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

import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.SolverData;
import box2d.dynamics.World;
import box2d.dynamics.contacts.Position;
import box2d.dynamics.contacts.Velocity;
import haxe.ds.Vector;

 class ConstantVolumeJoint extends Joint {

  private var bodies : Vector<Body>;
  private var targetLengths : Vector<Float>;
  private var targetVolume : Float;

  private var normals : Vector<Vec2>;
  private var m_impulse : Float = 0.0;

  private var world : World;

  private var distanceJoints : Vector<DistanceJoint>;

  public function getBodies() : Vector<Body> {
    return bodies;
  }

  public function getJoints() : Vector<DistanceJoint> {
    return distanceJoints;
  }

  public function inflate(factor : Float) : Void {
    targetVolume *= factor;
  }

  public function new(argWorld : World, def : ConstantVolumeJointDef) {
    super(argWorld.getPool(), def);
    world = argWorld;
    if (def.bodies.length <= 2) {
      throw "You cannot create a constant volume joint with less than three bodies.";
    }
    // TODO: list to Array?
    // bodies = def.bodies.toArray(new Array<Body>());

    targetLengths = new Vector<Float>(bodies.length);
    for (i in 0 ... targetLengths.length) {
    // for (int i = 0; i < targetLengths.length; ++i) {
      var next : Int = (i == targetLengths.length - 1) ? 0 : i + 1;
      var dist : Float = bodies[i].getWorldCenter().sub(bodies[next].getWorldCenter()).length();
      targetLengths[i] = dist;
    }
    targetVolume = getBodyArea();

    if (def.joints != null && def.joints.length != def.bodies.length) {
      throw "Incorrect joint definition.  Joints have to correspond to the bodies";
    }
    if (def.joints == null) {
      var djd : DistanceJointDef = new DistanceJointDef();
      distanceJoints = new Vector<DistanceJoint>(bodies.length);
      for (i in 0 ... targetLengths.length) {
      // for (int i = 0; i < targetLengths.length; ++i) {
        var next : Int = (i == targetLengths.length - 1) ? 0 : i + 1;
        djd.frequencyHz = def.frequencyHz;// 20.0;
        djd.dampingRatio = def.dampingRatio;// 50.0;
        djd.collideConnected = def.collideConnected;
        djd.initialize(bodies[i], bodies[next], bodies[i].getWorldCenter(),
            bodies[next].getWorldCenter());
        distanceJoints[i] = cast world.createJoint(djd);
      }
    } else {
      // TODO: list to Array?
      // distanceJoints = def.joints.toArray(new Vector());
    }

    normals = new Vector<Vec2>(bodies.length);
    for (i in 0 ... normals.length) {
    // for (int i = 0; i < normals.length; ++i) {
      normals[i] = new Vec2();
    }
  }

  override public function destructor() : Void {
    for (i in 0 ... distanceJoints.length) {
    // for (int i = 0; i < distanceJoints.length; ++i) {
      world.destroyJoint(distanceJoints[i]);
    }
  }

  private function getBodyArea() : Float {
    var area : Float = 0.0;
    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      area +=
          bodies[i].getWorldCenter().x * bodies[next].getWorldCenter().y
              - bodies[next].getWorldCenter().x * bodies[i].getWorldCenter().y;
    }
    area *= .5;
    return area;
  }

  private function getSolverArea(positions : Vector<Position>) : Float {
    var area : Float = 0.0;
    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      area +=
          positions[bodies[i].m_islandIndex].c.x * positions[bodies[next].m_islandIndex].c.y
              - positions[bodies[next].m_islandIndex].c.x * positions[bodies[i].m_islandIndex].c.y;
    }
    area *= .5;
    return area;
  }

  private function constrainEdges(positions : Vector<Position>) : Bool {
    var perimeter : Float = 0.0;
    for (i in 0 ... bodies.length) {
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      var dx : Float = positions[bodies[next].m_islandIndex].c.x - positions[bodies[i].m_islandIndex].c.x;
      var dy : Float = positions[bodies[next].m_islandIndex].c.y - positions[bodies[i].m_islandIndex].c.y;
      var dist : Float = MathUtils.sqrt(dx * dx + dy * dy);
      if (dist < Settings.EPSILON) {
        dist = 1.0;
      }
      normals[i].x = dy / dist;
      normals[i].y = -dx / dist;
      perimeter += dist;
    }

    var delta : Vec2 = pool.popVec2();

    var deltaArea : Float = targetVolume - getSolverArea(positions);
    var toExtrude : Float = 0.5 * deltaArea / perimeter; // *relaxationFactor
    // float sumdeltax = 0.0f;
    var done : Bool = true;
    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      delta.set(toExtrude * (normals[i].x + normals[next].x), toExtrude
          * (normals[i].y + normals[next].y));
      // sumdeltax += dx;
      var normSqrd : Float = delta.lengthSquared();
      if (normSqrd > Settings.maxLinearCorrection * Settings.maxLinearCorrection) {
        delta.mulLocal(Settings.maxLinearCorrection / MathUtils.sqrt(normSqrd));
      }
      if (normSqrd > Settings.linearSlop * Settings.linearSlop) {
        done = false;
      }
      positions[bodies[next].m_islandIndex].c.x += delta.x;
      positions[bodies[next].m_islandIndex].c.y += delta.y;
      // bodies[next].m_linearVelocity.x += delta.x * step.inv_dt;
      // bodies[next].m_linearVelocity.y += delta.y * step.inv_dt;
    }

    pool.pushVec2(1);
    // System.out.println(sumdeltax);
    return done;
  }

  override public function initVelocityConstraints(step : SolverData) : Void {
    var velocities : Vector<Velocity> = step.velocities;
    var positions : Vector<Position> = step.positions;
    
    var d : Vector<Vec2> = pool.getVec2Array(bodies.length);

    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      var prev : Int = (i == 0) ? bodies.length - 1 : i - 1;
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      d[i].setVec(positions[bodies[next].m_islandIndex].c);
      d[i].subLocal(positions[bodies[prev].m_islandIndex].c);
    }

    if (step.step.warmStarting) {
      m_impulse *= step.step.dtRatio;
      // float lambda = -2.0f * crossMassSum / dotMassSum;
      // System.out.println(crossMassSum + " " +dotMassSum);
      // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
      // Settings.maxLinearCorrection);
      // m_impulse = lambda;
      for (i in 0 ... bodies.length) {
      // for (int i = 0; i < bodies.length; ++i) {
        velocities[bodies[i].m_islandIndex].v.x += bodies[i].m_invMass * d[i].y * .5 * m_impulse;
        velocities[bodies[i].m_islandIndex].v.y += bodies[i].m_invMass * -d[i].x * .5 * m_impulse;
      }
    } else {
      m_impulse = 0.0;
    }
  }

  override public function solvePositionConstraints(step : SolverData) : Bool {
    return constrainEdges(step.positions);
  }

  override public function solveVelocityConstraints(step : SolverData) : Void {
    var crossMassSum : Float = 0.0;
    var dotMassSum : Float = 0.0;

    var velocities : Vector<Velocity> = step.velocities;
    var positions : Vector<Position> = step.positions;
    var d : Vector<Vec2> = pool.getVec2Array(bodies.length);

    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      var prev : Int = (i == 0) ? bodies.length - 1 : i - 1;
      var next : Int = (i == bodies.length - 1) ? 0 : i + 1;
      d[i].setVec(positions[bodies[next].m_islandIndex].c);
      d[i].subLocal(positions[bodies[prev].m_islandIndex].c);
      dotMassSum += (d[i].lengthSquared()) / bodies[i].getMass();
      crossMassSum += Vec2.crossVec(velocities[bodies[i].m_islandIndex].v, d[i]);
    }
    var lambda : Float = -2.0 * crossMassSum / dotMassSum;
    // System.out.println(crossMassSum + " " +dotMassSum);
    // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
    // Settings.maxLinearCorrection);
    m_impulse += lambda;
    // System.out.println(m_impulse);
    for (i in 0 ... bodies.length) {
    // for (int i = 0; i < bodies.length; ++i) {
      velocities[bodies[i].m_islandIndex].v.x += bodies[i].m_invMass * d[i].y * .5 * lambda;
      velocities[bodies[i].m_islandIndex].v.y += bodies[i].m_invMass * -d[i].x * .5 * lambda;
    }
  }

  /** No-op */
  override public function getAnchorA(argOut:Vec2) : Void {}

  /** No-op */
  override public function getAnchorB(argOut:Vec2) : Void {}

  /** No-op */
  override public function getReactionForce(inv_dt : Float, argOut : Vec2) : Void {}

  /** No-op */
  override public function getReactionTorque(inv_dt : Float) : Float {
    return 0;
  }
}

