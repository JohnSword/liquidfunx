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

/**
 * Prismatic joint definition. This requires defining a line of motion using an axis and an anchor
 * point. The definition uses local anchor points and a local axis so that the initial configuration
 * can violate the constraint slightly. The joint translation is zero when the local anchor points
 * coincide in world space. Using local anchors and a local axis helps when saving and loading a
 * game.
 * 
 * @warning at least one body should by dynamic with a non-fixed rotation.
 * @author Daniel
 * 
 */
 class PrismaticJointDef extends JointDef {


  /**
   * The local anchor point relative to body1's origin.
   */
  public var localAnchorA : Vec2;

  /**
   * The local anchor point relative to body2's origin.
   */
  public var localAnchorB : Vec2;

  /**
   * The local translation axis in body1.
   */
  public var localAxisA : Vec2;

  /**
   * The constrained angle between the bodies: body2_angle - body1_angle.
   */
  public var referenceAngle : Float;

  /**
   * Enable/disable the joint limit.
   */
  public var enableLimit : Bool;

  /**
   * The lower translation limit, usually in meters.
   */
  public var lowerTranslation : Float;

  /**
   * The upper translation limit, usually in meters.
   */
  public var upperTranslation : Float;

  /**
   * Enable/disable the joint motor.
   */
  public var enableMotor : Bool;

  /**
   * The maximum motor torque, usually in N-m.
   */
  public var maxMotorForce : Float;

  /**
   * The desired motor speed in radians per second.
   */
  public var motorSpeed : Float;

  public function new() {
    super(JointType.PRISMATIC);
    localAnchorA = new Vec2();
    localAnchorB = new Vec2();
    localAxisA = new Vec2(1.0, 0.0);
    referenceAngle = 0.0;
    enableLimit = false;
    lowerTranslation = 0.0;
    upperTranslation = 0.0;
    enableMotor = false;
    maxMotorForce = 0.0;
    motorSpeed = 0.0;
  }


  /**
   * Initialize the bodies, anchors, axis, and reference angle using the world anchor and world
   * axis.
   */
  public function initialize(b1 : Body, b2 : Body, anchor : Vec2, axis : Vec2) : Void {
    bodyA = b1;
    bodyB = b2;
    bodyA.getLocalPointToOut(anchor, localAnchorA);
    bodyB.getLocalPointToOut(anchor, localAnchorB);
    bodyA.getLocalVectorToOut(axis, localAxisA);
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}

