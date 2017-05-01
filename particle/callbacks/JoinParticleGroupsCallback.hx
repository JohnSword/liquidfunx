package box2d.particle.callbacks;

import box2d.common.Vec2;
import box2d.common.Settings;
import box2d.common.BufferUtils;
import box2d.common.MathUtils;
import box2d.particle.ParticleSystem;

// Callback used with VoronoiDiagram.
class JoinParticleGroupsCallback implements VoronoiDiagramCallback {

    public function new() {}

    public function callback(a : Int, b : Int, c : Int) : Void {
      // Create a triad if it will contain particles from both groups.
      var countA : Int =
          ((a < groupB.m_firstIndex) ? 1 : 0) + ((b < groupB.m_firstIndex) ? 1 : 0)
              + ((c < groupB.m_firstIndex) ? 1 : 0);
      if (countA > 0 && countA < 3) {
        var af : Int = system.m_flagsBuffer.data[a];
        var bf : Int = system.m_flagsBuffer.data[b];
        var cf : Int = system.m_flagsBuffer.data[c];
        if ((af & bf & cf & ParticleSystem.k_triadFlags) != 0) {
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
            triad.flags = af | bf | cf;
            triad.strength = MathUtils.min(groupA.m_strength, groupB.m_strength);
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
    }

    public var system : ParticleSystem;
    public var groupA : ParticleGroup;
    public var groupB : ParticleGroup;
  }