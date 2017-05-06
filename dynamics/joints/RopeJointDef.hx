package box2d.dynamics.joints;

import box2d.common.Vec2;

/**
 * Rope joint definition. This requires two body anchor points and a maximum lengths. Note: by
 * default the connected objects will not collide. see collideConnected in b2JointDef.
 * 
 * @author Daniel Murphy
 */
 class RopeJointDef extends JointDef {

  /**
   * The local anchor point relative to bodyA's origin.
   */
  public var localAnchorA : Vec2 = new Vec2();

  /**
   * The local anchor point relative to bodyB's origin.
   */
  public var localAnchorB : Vec2 = new Vec2();

  /**
   * The maximum length of the rope. Warning: this must be larger than b2_linearSlop or the joint
   * will have no effect.
   */
  public var maxLength : Float = 0;

  public function new() {
    super(JointType.ROPE);
    localAnchorA.set(-1.0, 0.0);
    localAnchorB.set(1.0, 0.0);
  }
}

