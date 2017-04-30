package box2d.particle;

class ParticleBuffer<T> {
    public var data : Array<T>;
    public var dataClass : Array<Dynamic>;
    public var userSuppliedCapacity : Int;

    public function new(dataClass : Array<T>) {
      this.dataClass = dataClass;
    }
  }