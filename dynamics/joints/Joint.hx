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

import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.SolverData;
import box2d.dynamics.World;
import box2d.pooling.IWorldPool;

// updated to rev 100
/**
 * The base joint class. Joints are used to constrain two bodies together in various fashions. Some
 * joints also feature limits and motors.
 * 
 * @author Daniel Murphy
 */
class Joint {

  public static function create(world : World, def : JointDef) : Joint {
    // Joint joint = null;
    switch (def.type) {
      case MOUSE:
        return new MouseJoint(world.getPool(), cast def);
      case DISTANCE:
        return new DistanceJoint(world.getPool(), cast def);
      case PRISMATIC:
        return new PrismaticJoint(world.getPool(), cast def);
      case REVOLUTE:
        return new RevoluteJoint(world.getPool(), cast def);
      case WELD:
        return new WeldJoint(world.getPool(), cast def);
      case FRICTION:
        return new FrictionJoint(world.getPool(), cast def);
      case WHEEL:
        return new WheelJoint(world.getPool(), cast def);
      case GEAR:
        return new GearJoint(world.getPool(), cast def);
      case PULLEY:
        return new PulleyJoint(world.getPool(), cast def);
      case CONSTANT_VOLUME:
        return new ConstantVolumeJoint(world, cast def);
      case ROPE:
        return new RopeJoint(world.getPool(), cast def);
      case MOTOR:
        return new MotorJoint(world.getPool(), cast def);
      case UNKNOWN:
        return null;
      default:
        return null;
    }
  }

  public static function destroy(joint : Joint) : Void {
    joint.destructor();
  }

  private var m_type : JointType;
  public var m_prev : Joint;
  public var m_next : Joint;
  public var m_edgeA : JointEdge;
  public var m_edgeB : JointEdge;
  public var m_bodyA : Body;
  public var m_bodyB : Body;

  public var m_islandFlag : Bool;
  private var m_collideConnected : Bool;

  public var m_userData : Dynamic;

  public var pool : IWorldPool;

  // Cache here per time step to reduce cache misses.
  // final Vec2 m_localCenterA, m_localCenterB;
  // float m_invMassA, m_invIA;
  // float m_invMassB, m_invIB;

  public function new(worldPool : IWorldPool, def : JointDef) {

    pool = worldPool;
    m_type = def.type;
    m_prev = null;
    m_next = null;
    m_bodyA = def.bodyA;
    m_bodyB = def.bodyB;
    m_collideConnected = def.collideConnected;
    m_islandFlag = false;
    m_userData = def.userData;

    m_edgeA = new JointEdge();
    m_edgeA.joint = null;
    m_edgeA.other = null;
    m_edgeA.prev = null;
    m_edgeA.next = null;

    m_edgeB = new JointEdge();
    m_edgeB.joint = null;
    m_edgeB.other = null;
    m_edgeB.prev = null;
    m_edgeB.next = null;

    // m_localCenterA = new Vec2();
    // m_localCenterB = new Vec2();
  }

  /**
   * get the type of the concrete joint.
   * 
   * @return
   */
  public function getType() : JointType {
    return m_type;
  }

  /**
   * get the first body attached to this joint.
   */
  public function getBodyA() : Body {
    return m_bodyA;
  }

  /**
   * get the second body attached to this joint.
   * 
   * @return
   */
  public function getBodyB() : Body {
    return m_bodyB;
  }

  /**
   * get the anchor point on bodyA in world coordinates.
   * 
   * @return
   */
  public function getAnchorA(out:Vec2) : Void {

  }

  /**
   * get the anchor point on bodyB in world coordinates.
   * 
   * @return
   */
  public function getAnchorB(out:Vec2) : Void {
    
  }

  /**
   * get the reaction force on body2 at the joint anchor in Newtons.
   * 
   * @param inv_dt
   * @return
   */
  public function getReactionForce(inv_dt:Float, out:Vec2) : Void {
    
  }

  /**
   * get the reaction torque on body2 in N*m.
   * 
   * @param inv_dt
   * @return
   */
  public function getReactionTorque(inv_dt:Float) : Void {
    
  }

  /**
   * get the next joint the world joint list.
   */
  public function getNext() : Joint {
    return m_next;
  }

  /**
   * get the user data pointer.
   */
  public function getUserData() : Dynamic {
    return m_userData;
  }

  /**
   * Set the user data pointer.
   */
  public function setUserData(data : Dynamic) : Void {
    m_userData = data;
  }

  /**
   * Get collide connected. Note: modifying the collide connect flag won't work correctly because
   * the flag is only checked when fixture AABBs begin to overlap.
   */
  public function getCollideConnected() : Bool {
    return m_collideConnected;
  }

  /**
   * Short-cut function to determine if either body is inactive.
   * 
   * @return
   */
  public function isActive() : Bool {
    return m_bodyA.isActive() && m_bodyB.isActive();
  }

  /** Internal */
  public function initVelocityConstraints(data:SolverData) : Void {

  }

  /** Internal */
  public function solveVelocityConstraints(data:SolverData) : Void {
    
  }

  /**
   * This returns true if the position errors are within tolerance. Internal.
   */
  public function solvePositionConstraints(data:SolverData) : Bool {
    return false;
  }

  /**
   * Override to handle destruction of joint
   */
  public function destructor() : Void {}
}

