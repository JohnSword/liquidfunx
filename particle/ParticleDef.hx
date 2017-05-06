package box2d.particle;

import box2d.common.Vec2;

 class ParticleDef {
  /**
   * Specifies the type of particle. A particle may be more than one type. Multiple types are
   * chained by logical sums, for example: pd.flags = ParticleType.b2_elasticParticle |
   * ParticleType.b2_viscousParticle.
   */
  public var flags : Int = 0;

  /** The world position of the particle. */
  public var position : Vec2 = new Vec2();

  /** The linear velocity of the particle in world co-ordinates. */
  public var velocity : Vec2 = new Vec2();

  /** The color of the particle. */
  public var color : ParticleColor;

  /** Use this to store application-specific body data. */
  public var userData : Dynamic;

  public function new() {}
}

