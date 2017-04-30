package box2d.particle;

import box2d.common.Vec2;

 class ParticleContact {
  /** Indices of the respective particles making contact. */
  public var indexB : Int;
  public var indexA : Int;
  /** The logical sum of the particle behaviors that have been set. */
  public var flags : Int;
  /** Weight of the contact. A value between 0.0f and 1.0f. */
  public var weight : Float;
  /** The normalized direction from A to B. */
  public var normal : Vec2 = new Vec2();

  public function new() {}
}

