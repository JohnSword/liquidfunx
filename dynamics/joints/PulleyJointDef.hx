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
/**
 * Created at 12:11:41 PM Jan 23, 2011
 */
package box2d.dynamics.joints;

import box2d.common.Vec2;
import box2d.dynamics.Body;

/**
 * Pulley joint definition. This requires two ground anchors, two dynamic body anchor points, and a
 * pulley ratio.
 * 
 * @author Daniel Murphy
 */
 class PulleyJointDef extends JointDef {

  /**
   * The first ground anchor in world coordinates. This point never moves.
   */
  public var groundAnchorA : Vec2;

  /**
   * The second ground anchor in world coordinates. This point never moves.
   */
  public var groundAnchorB : Vec2;

  /**
   * The local anchor point relative to bodyA's origin.
   */
  public var localAnchorA : Vec2;

  /**
   * The local anchor point relative to bodyB's origin.
   */
  public var localAnchorB : Vec2;

  /**
   * The a reference length for the segment attached to bodyA.
   */
  public var lengthA : Float = 0;

  /**
   * The a reference length for the segment attached to bodyB.
   */
  public var lengthB : Float = 0;

  /**
   * The pulley ratio, used to simulate a block-and-tackle.
   */
  public var ratio : Float;

  public function new() {
    super(JointType.PULLEY);
    groundAnchorA = new Vec2(-1.0, 1.0);
    groundAnchorB = new Vec2(1.0, 1.0);
    localAnchorA = new Vec2(-1.0, 0.0);
    localAnchorB = new Vec2(1.0, 0.0);
    lengthA = 0.0;
    lengthB = 0.0;
    ratio = 1.0;
    collideConnected = true;
  }

  /**
   * Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
   */
  public function initialize(b1 : Body, b2 : Body, ga1 : Vec2, ga2 : Vec2, anchor1 : Vec2, anchor2 : Vec2, r : Float) : Void {
    bodyA = b1;
    bodyB = b2;
    groundAnchorA = ga1;
    groundAnchorB = ga2;
    localAnchorA = bodyA.getLocalPoint(anchor1);
    localAnchorB = bodyB.getLocalPoint(anchor2);
    var d1 : Vec2 = anchor1.sub(ga1);
    lengthA = d1.length();
    var d2 : Vec2 = anchor2.sub(ga2);
    lengthB = d2.length();
    ratio = r;
  }
}

