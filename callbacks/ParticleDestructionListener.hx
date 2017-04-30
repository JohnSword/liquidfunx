package box2d.callbacks;

import box2d.particle.ParticleGroup;

interface ParticleDestructionListener {
  /**
   * Called when any particle group is about to be destroyed.
   */
  function sayGoodbyeParticleGroup(group : ParticleGroup) : Void;

  /**
   * Called when a particle is about to be destroyed. The index can be used in conjunction with
   * {@link World#getParticleUserDataBuffer} to determine which particle has been destroyed.
   * 
   * @param index
   */
  function sayGoodbyeIndex(index : Int) : Void;
}