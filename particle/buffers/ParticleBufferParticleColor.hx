package box2d.particle.buffers;

import box2d.particle.ParticleColor;
import haxe.ds.Vector;

class ParticleBufferParticleColor {
    public var data : Vector<ParticleColor>;
    public var dataClass : Class<Dynamic>;
    public var userSuppliedCapacity : Int = 0;

    public function new(dataClass : Class<Dynamic>) {
      this.dataClass = dataClass;
    }
  }