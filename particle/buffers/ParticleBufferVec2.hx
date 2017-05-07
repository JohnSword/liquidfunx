package box2d.particle.buffers;

import box2d.common.Vec2;
import haxe.ds.Vector;

class ParticleBufferVec2 {
    public var data : Vector<Vec2>;
    public var dataClass : Class<Dynamic>;
    public var userSuppliedCapacity : Int = 0;

    public function new(dataClass : Class<Dynamic>) {
      this.dataClass = dataClass;
    }
  }