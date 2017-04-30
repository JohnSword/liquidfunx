package box2d.particle;

import box2d.collision.shapes.Shape;
import box2d.common.Vec2;

/**
 * A particle group definition holds all the data needed to construct a particle group. You can
 * safely re-use these definitions.
 */
 class ParticleGroupDef {

  /** The particle-behavior flags. */
  public var flags : Int;

  /** The group-construction flags. */
  public var groupFlags : Int;

  /**
   * The world position of the group. Moves the group's shape a distance equal to the value of
   * position.
   */
  public var position : Vec2 = new Vec2();

  /**
   * The world angle of the group in radians. Rotates the shape by an angle equal to the value of
   * angle.
   */
  public var angle : Float;

  /** The linear velocity of the group's origin in world co-ordinates. */
  public var linearVelocity : Vec2 = new Vec2();

  /** The angular velocity of the group. */
  public var angularVelocity : Float;

  /** The color of all particles in the group. */
  public var color : ParticleColor;

  /**
   * The strength of cohesion among the particles in a group with flag b2_elasticParticle or
   * b2_springParticle.
   */
  public var strength : Float;

  /** Shape containing the particle group. */
  public var shape : Shape;

  /** If true, destroy the group automatically after its last particle has been destroyed. */
  public var destroyAutomatically : Bool;

  /** Use this to store application-specific group data. */
  public var userData : Dynamic;

  public function new() {
    flags = 0;
    groupFlags = 0;
    angle = 0;
    angularVelocity = 0;
    strength = 1;
    destroyAutomatically = true;
  }
}

