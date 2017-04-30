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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package box2d.dynamics.joints;

import box2d.common.Vec2;
import box2d.dynamics.Body;

/**
 * Revolute joint definition. This requires defining an anchor point where the bodies are joined.
 * The definition uses local anchor points so that the initial configuration can violate the
 * constraint slightly. You also need to specify the initial relative angle for joint limits. This
 * helps when saving and loading a game. The local anchor points are measured from the body's origin
 * rather than the center of mass because:<br/>
 * <ul>
 * <li>you might not know where the center of mass will be.</li>
 * <li>if you add/remove shapes from a body and recompute the mass, the joints will be broken.</li>
 * </ul>
 */
 class RevoluteJointDef extends JointDef {

  /**
   * The local anchor point relative to body1's origin.
   */
  public var localAnchorA : Vec2;

  /**
   * The local anchor point relative to body2's origin.
   */
  public var localAnchorB : Vec2;

  /**
   * The body2 angle minus body1 angle in the reference state (radians).
   */
  public var referenceAngle : Float;

  /**
   * A flag to enable joint limits.
   */
  public var enableLimit : Bool;

  /**
   * The lower angle for the joint limit (radians).
   */
  public var lowerAngle : Float;

  /**
   * The upper angle for the joint limit (radians).
   */
  public var upperAngle : Float;

  /**
   * A flag to enable the joint motor.
   */
  public var enableMotor : Bool;

  /**
   * The desired motor speed. Usually in radians per second.
   */
  public var motorSpeed : Float;

  /**
   * The maximum motor torque used to achieve the desired motor speed. Usually in N-m.
   */
  public var maxMotorTorque : Float;

  public function new() {
    super(JointType.REVOLUTE);
    localAnchorA = new Vec2(0.0, 0.0);
    localAnchorB = new Vec2(0.0, 0.0);
    referenceAngle = 0.0;
    lowerAngle = 0.0;
    upperAngle = 0.0;
    maxMotorTorque = 0.0;
    motorSpeed = 0.0;
    enableLimit = false;
    enableMotor = false;
  }

  /**
   * Initialize the bodies, anchors, and reference angle using the world anchor.
   * 
   * @param b1
   * @param b2
   * @param anchor
   */
  public function initialize(b1 : Body, b2 : Body, anchor : Vec2) : Void {
    bodyA = b1;
    bodyB = b2;
    bodyA.getLocalPointToOut(anchor, localAnchorA);
    bodyB.getLocalPointToOut(anchor, localAnchorB);
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}

