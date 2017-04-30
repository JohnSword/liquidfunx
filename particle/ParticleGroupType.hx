package box2d.particle;

 class ParticleGroupType {
  /** resists penetration */
  public static var b2_solidParticleGroup : Int = 1 << 0;
  /** keeps its shape */
  public static var b2_rigidParticleGroup : Int = 1 << 1;
}

