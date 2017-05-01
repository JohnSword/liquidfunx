package box2d.particle;

import haxe.ds.Vector;

class ParticleBuffer {
    public var data : Vector<Dynamic>;
    public var dataClass : Class<Dynamic>;
    public var userSuppliedCapacity : Int;

    public function new(dataClass : Class<Dynamic>) {
      this.dataClass = dataClass;
    }
  }