package box2d.pooling.arrays;

import haxe.ds.Vector;
import box2d.particle.Generator;

 class GeneratorArray {

  private var map : Map<Int, Vector<Generator>> = new Map<Int, Vector<Generator>>();

  public function get(length : Int) : Vector<Generator> {
    if (!map.exists(length)) {
      map.set(length, getInitializedArray(length));
    }
    return map.get(length);
  }

  public function getInitializedArray(length : Int) : Vector<Generator> {
    var ray : Vector<Generator> = new Vector<Generator>(length);
    for(i in 0 ... ray.length) {
      ray[i] = new Generator();
    }
    return ray;
  }
}

