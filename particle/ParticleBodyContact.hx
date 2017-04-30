package box2d.particle;

import box2d.common.Vec2;
import box2d.dynamics.Body;

 class ParticleBodyContact {
  /** Index of the particle making contact. */
  public var index : Int;
  /** The body making contact. */
  public var body : Body;
  /** Weight of the contact. A value between 0.0f and 1.0f. */
  public var weight : Float;
  /** The normalized direction from the particle to the body. */
  public var normal : Vec2 = new Vec2();
  /** The effective mass used in calculating force. */
  public var mass : Float;
  
  public function new() {}
}

