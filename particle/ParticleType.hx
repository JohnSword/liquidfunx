package box2d.particle;

/**
 * The particle type. Can be combined with | operator. Zero means liquid.
 * 
 * @author dmurph
 */
 class ParticleType {
  public static var b2_waterParticle : Int = 0;
  /** removed after next step */
  public static var b2_zombieParticle : Int = 1 << 1;
  /** zero velocity */
  public static var b2_wallParticle : Int = 1 << 2;
  /** with restitution from stretching */
  public static var b2_springParticle : Int = 1 << 3;
  /** with restitution from deformation */
  public static var b2_elasticParticle : Int = 1 << 4;
  /** with viscosity */
  public static var b2_viscousParticle : Int = 1 << 5;
  /** without isotropic pressure */
  public static var b2_powderParticle : Int = 1 << 6;
  /** with surface tension */
  public static var b2_tensileParticle : Int = 1 << 7;
  /** mixing color between contacting particles */
  public static var b2_colorMixingParticle : Int = 1 << 8;
  /** call b2DestructionListener on destruction */
  public static var b2_destructionListener : Int = 1 << 9;
}

