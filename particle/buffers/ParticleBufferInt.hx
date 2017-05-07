package box2d.particle.buffers;

import haxe.ds.Vector;

class ParticleBufferInt {
   public var data : Vector<Int>;
   public var userSuppliedCapacity : Int = 0;
   public function new() {}
}