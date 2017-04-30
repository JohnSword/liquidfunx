package box2d.callbacks;

import box2d.common.Vec2;

interface ParticleRaycastCallback {
  /**
   * Called for each particle found in the query. See
   * {@link RayCastCallback#reportFixture(org.jbox2d.dynamics.Fixture, Vec2, Vec2, float)} for
   * argument info.
   * 
   * @param index
   * @param point
   * @param normal
   * @param fraction
   * @return
   */
  function reportParticle(index : Int, point : Vec2, normal : Vec2, fraction : Float) : Float;

}