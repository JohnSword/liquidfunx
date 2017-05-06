package box2d.particle.callbacks;

import box2d.common.Vec2;
import box2d.common.Settings;
import box2d.common.BufferUtils;

// Callback used with VoronoiDiagram.
class CreateParticleGroupCallback implements VoronoiDiagramCallback {

    public var system : ParticleSystem;
    public var def : ParticleGroupDef; // pointer
    public var firstIndex : Int = 0;

    public function new() {}

    public function callback(a : Int, b : Int, c : Int) : Void {
      var pa : Vec2 = system.m_positionBuffer.data[a];
      var pb : Vec2 = system.m_positionBuffer.data[b];
      var pc : Vec2 = system.m_positionBuffer.data[c];
      var dabx : Float = pa.x - pb.x;
      var daby : Float = pa.y - pb.y;
      var dbcx : Float = pb.x - pc.x;
      var dbcy : Float = pb.y - pc.y;
      var dcax : Float = pc.x - pa.x;
      var dcay : Float = pc.y - pa.y;
      var maxDistanceSquared : Float = Settings.maxTriadDistanceSquared * system.m_squaredDiameter;
      if (dabx * dabx + daby * daby < maxDistanceSquared
          && dbcx * dbcx + dbcy * dbcy < maxDistanceSquared
          && dcax * dcax + dcay * dcay < maxDistanceSquared) {
        if (system.m_triadCount >= system.m_triadCapacity) {
          var oldCapacity : Int = system.m_triadCapacity;
          var newCapacity : Int =
              system.m_triadCount != 0
                  ? 2 * system.m_triadCount
                  : Settings.minParticleBufferCapacity;
          system.m_triadBuffer =
              cast BufferUtils.reallocateBuffer(Triad, system.m_triadBuffer, oldCapacity,
                  newCapacity);
          system.m_triadCapacity = newCapacity;
        }
        var triad : Triad = system.m_triadBuffer[system.m_triadCount];
        triad.indexA = a;
        triad.indexB = b;
        triad.indexC = c;
        triad.flags =
            system.m_flagsBuffer.data[a] | system.m_flagsBuffer.data[b]
                | system.m_flagsBuffer.data[c];
        triad.strength = def.strength;
        var midPointx : Float = 1 / 3 * (pa.x + pb.x + pc.x);
        var midPointy : Float = 1 / 3 * (pa.y + pb.y + pc.y);
        triad.pa.x = pa.x - midPointx;
        triad.pa.y = pa.y - midPointy;
        triad.pb.x = pb.x - midPointx;
        triad.pb.y = pb.y - midPointy;
        triad.pc.x = pc.x - midPointx;
        triad.pc.y = pc.y - midPointy;
        triad.ka = -(dcax * dabx + dcay * daby);
        triad.kb = -(dabx * dbcx + daby * dbcy);
        triad.kc = -(dbcx * dcax + dbcy * dcay);
        triad.s = Vec2.crossVec(pa, pb) + Vec2.crossVec(pb, pc) + Vec2.crossVec(pc, pa);
        system.m_triadCount++;
      }
    }
  }