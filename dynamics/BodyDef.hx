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

import box2d.common.Vec2;

/**
 * A body definition holds all the data needed to construct a rigid body. You can safely re-use body
 * definitions. Shapes are added to a body after construction.
 * 
 * @author daniel
 */
 class BodyDef {

  /**
   * The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
   * mass is set to one.
   */
  public var type : BodyType;

  /**
   * Use this to store application specific body data.
   */
  public var userData : Dynamic;

  /**
   * The world position of the body. Avoid creating bodies at the origin since this can lead to many
   * overlapping shapes.
   */
  public var position : Vec2;

  /**
   * The world angle of the body in radians.
   */
  public var angle : Float = 0;

  /**
   * The linear velocity of the body in world co-ordinates.
   */
  public var linearVelocity : Vec2;

  /**
   * The angular velocity of the body.
   */
  public var angularVelocity : Float = 0;

  /**
   * Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public var linearDamping : Float = 0;

  /**
   * Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public var angularDamping : Float = 0;

  /**
   * Set this flag to false if this body should never fall asleep. Note that this increases CPU
   * usage.
   */
  public var allowSleep : Bool;

  /**
   * Is this body initially sleeping?
   */
  public var awake : Bool;

  /**
   * Should this body be prevented from rotating? Useful for characters.
   */
  public var fixedRotation : Bool;

  /**
   * Is this a fast moving body that should be prevented from tunneling through other moving bodies?
   * Note that all bodies are prevented from tunneling through kinematic and static bodies. This
   * setting is only considered on dynamic bodies.
   * 
   * @warning You should use this flag sparingly since it increases processing time.
   */
  public var bullet : Bool;

  /**
   * Does this body start out active?
   */
  public var active : Bool;

  /**
   * Experimental: scales the inertia tensor.
   */
  public var gravityScale : Float = 0;

  public function new() {
    userData = null;
    position = new Vec2();
    angle = 0;
    linearVelocity = new Vec2();
    angularVelocity = 0;
    linearDamping = 0;
    angularDamping = 0;
    allowSleep = true;
    awake = true;
    fixedRotation = false;
    bullet = false;
    type = BodyType.STATIC;
    active = true;
    gravityScale = 1.0;
  }

  /**
   * The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
   * mass is set to one.
   */
  public function getType() : BodyType {
    return type;
  }

  /**
   * The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
   * mass is set to one.
   */
  public function setType(type : BodyType) : Void {
    this.type = type;
  }

  /**
   * Use this to store application specific body data.
   */
  public function getUserData() : Dynamic {
    return userData;
  }

  /**
   * Use this to store application specific body data.
   */
  public function setUserData(userData : Dynamic) : Void {
    this.userData = userData;
  }

  /**
   * The world position of the body. Avoid creating bodies at the origin since this can lead to many
   * overlapping shapes.
   */
  public function getPosition() : Vec2 {
    return position;
  }

  /**
   * The world position of the body. Avoid creating bodies at the origin since this can lead to many
   * overlapping shapes.
   */
  public function setPosition(position : Vec2) : Void {
    this.position = position;
  }

  /**
   * The world angle of the body in radians.
   */
  public function getAngle() : Float {
    return angle;
  }

  /**
   * The world angle of the body in radians.
   */
  public function setAngle(angle : Float) : Void {
    this.angle = angle;
  }

  /**
   * The linear velocity of the body in world co-ordinates.
   */
  public function getLinearVelocity() : Vec2 {
    return linearVelocity;
  }

  /**
   * The linear velocity of the body in world co-ordinates.
   */
  public function setLinearVelocity(linearVelocity : Vec2) : Void {
    this.linearVelocity = linearVelocity;
  }

  /**
   * The angular velocity of the body.
   */
  public function getAngularVelocity() : Float {
    return angularVelocity;
  }

  /**
   * The angular velocity of the body.
   */
  public function setAngularVelocity(angularVelocity : Float) : Void {
    this.angularVelocity = angularVelocity;
  }

  /**
   * Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public function getLinearDamping() : Float {
    return linearDamping;
  }

  /**
   * Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public function setLinearDamping(linearDamping : Float) : Void {
    this.linearDamping = linearDamping;
  }

  /**
   * Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public function getAngularDamping() : Float {
    return angularDamping;
  }

  /**
   * Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
   * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   */
  public function setAngularDamping(angularDamping : Float) : Void {
    this.angularDamping = angularDamping;
  }

  /**
   * Set this flag to false if this body should never fall asleep. Note that this increases CPU
   * usage.
   */
  public function isAllowSleep() : Bool {
    return allowSleep;
  }

  /**
   * Set this flag to false if this body should never fall asleep. Note that this increases CPU
   * usage.
   */
  public function setAllowSleep(allowSleep : Bool) : Void {
    this.allowSleep = allowSleep;
  }

  /**
   * Is this body initially sleeping?
   */
  public function isAwake() : Bool {
    return awake;
  }

  /**
   * Is this body initially sleeping?
   */
  public function setAwake(awake : Bool) : Void {
    this.awake = awake;
  }

  /**
   * Should this body be prevented from rotating? Useful for characters.
   */
  public function isFixedRotation() : Bool {
    return fixedRotation;
  }

  /**
   * Should this body be prevented from rotating? Useful for characters.
   */
  public function setFixedRotation(fixedRotation : Bool) : Void {
    this.fixedRotation = fixedRotation;
  }

  /**
   * Is this a fast moving body that should be prevented from tunneling through other moving bodies?
   * Note that all bodies are prevented from tunneling through kinematic and static bodies. This
   * setting is only considered on dynamic bodies.
   * 
   * @warning You should use this flag sparingly since it increases processing time.
   */
  public function isBullet() : Bool {
    return bullet;
  }

  /**
   * Is this a fast moving body that should be prevented from tunneling through other moving bodies?
   * Note that all bodies are prevented from tunneling through kinematic and static bodies. This
   * setting is only considered on dynamic bodies.
   * 
   * @warning You should use this flag sparingly since it increases processing time.
   */
  public function setBullet(bullet : Bool) : Void {
    this.bullet = bullet;
  }

  /**
   * Does this body start out active?
   */
  public function isActive() : Bool {
    return active;
  }

  /**
   * Does this body start out active?
   */
  public function setActive(active : Bool) : Void {
    this.active = active;
  }

  /**
   * Experimental: scales the inertia tensor.
   */
  public function getGravityScale() : Float {
    return gravityScale;
  }

  /**
   * Experimental: scales the inertia tensor.
   */
  public function setGravityScale(gravityScale : Float) : Void {
    this.gravityScale = gravityScale;
  }
}

