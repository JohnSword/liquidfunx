package box2d.particle.callbacks;

import box2d.collision.shapes.Shape;
import box2d.common.Transform;
import box2d.callbacks.ParticleQueryCallback;

class DestroyParticlesInShapeCallback implements ParticleQueryCallback {
    private var system : ParticleSystem;
    private var shape : Shape;
    private var xf : Transform;
    private var callDestructionListener : Bool;
    public var destroyed : Int;

    public function new() {
      // TODO Auto-generated constructor stub
    }

    public function init(system : ParticleSystem, shape : Shape, xf : Transform, callDestructionListener : Bool) : Void {
      this.system = system;
      this.shape = shape;
      this.xf = xf;
      this.destroyed = 0;
      this.callDestructionListener = callDestructionListener;
    }

    public function reportParticle(index : Int) : Bool {
      if (shape.testPoint(xf, system.m_positionBuffer.data[index])) {
        system.destroyParticle(index, callDestructionListener);
        destroyed++;
      }
      return true;
    }
  }