package box2d.particle.callbacks;

import box2d.dynamics.Fixture;
import box2d.common.Vec2;
import box2d.common.Transform;
import box2d.common.Settings;
import box2d.callbacks.QueryCallback;
import box2d.dynamics.TimeStep;
import box2d.collision.RayCastInput;
import box2d.collision.RayCastOutput;
import box2d.collision.AABB;
import box2d.collision.shapes.Shape;
import box2d.dynamics.Body;
import box2d.particle.ParticleSystem;

class SolveCollisionCallback implements QueryCallback {
    public var system : ParticleSystem;
    public var step : TimeStep;

    private var input : RayCastInput = new RayCastInput();
    private var output : RayCastOutput = new RayCastOutput();
    private var tempVec : Vec2 = new Vec2();
    private var tempVec2 : Vec2 = new Vec2();

    public function new() {}

    public function reportFixture(fixture : Fixture) : Bool {
      if (fixture.isSensor()) {
        return true;
      }
      var shape : Shape = fixture.getShape();
      var body : Body = fixture.getBody();
      var childCount : Int = shape.getChildCount();
      for (childIndex in 0 ... childCount) {
        var aabb : AABB = fixture.getAABB(childIndex);
        var aabblowerBoundx : Float = aabb.lowerBound.x - system.m_particleDiameter;
        var aabblowerBoundy : Float = aabb.lowerBound.y - system.m_particleDiameter;
        var aabbupperBoundx : Float = aabb.upperBound.x + system.m_particleDiameter;
        var aabbupperBoundy : Float = aabb.upperBound.y + system.m_particleDiameter;
        var firstProxy : Int =
            ParticleSystem.lowerBound(
                system.m_proxyBuffer,
                system.m_proxyCount,
                ParticleSystem.computeTag(system.m_inverseDiameter * aabblowerBoundx, system.m_inverseDiameter
                    * aabblowerBoundy));
        var lastProxy : Int =
            ParticleSystem.upperBound(
                system.m_proxyBuffer,
                system.m_proxyCount,
                ParticleSystem.computeTag(system.m_inverseDiameter * aabbupperBoundx, system.m_inverseDiameter
                    * aabbupperBoundy));
        
        var proxy : Int = firstProxy;
        while (proxy != lastProxy) {
        // for (int proxy = firstProxy; proxy != lastProxy; ++proxy) {
          var a : Int = system.m_proxyBuffer[proxy].index;
          var ap : Vec2 = system.m_positionBuffer.data[a];
          if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y
              && ap.y <= aabbupperBoundy) {
            var av : Vec2 = system.m_velocityBuffer.data[a];
            var temp : Vec2 = tempVec;
            Transform.mulTransToOutUnsafe(body.m_xf0, ap, temp);
            Transform.mulToOutUnsafe(body.m_xf, temp, input.p1);
            input.p2.x = ap.x + step.dt * av.x;
            input.p2.y = ap.y + step.dt * av.y;
            input.maxFraction = 1;
            if (fixture.raycast(output, input, childIndex)) {
              var p : Vec2 = tempVec;
              p.x =
                  (1 - output.fraction) * input.p1.x + output.fraction * input.p2.x
                      + Settings.linearSlop * output.normal.x;
              p.y =
                  (1 - output.fraction) * input.p1.y + output.fraction * input.p2.y
                      + Settings.linearSlop * output.normal.y;

              var vx : Float = step.inv_dt * (p.x - ap.x);
              var vy : Float = step.inv_dt * (p.y - ap.y);
              av.x = vx;
              av.y = vy;
              var particleMass : Float = system.getParticleMass();
              var ax : Float = particleMass * (av.x - vx);
              var ay : Float = particleMass * (av.y - vy);
              var b : Vec2 = output.normal;
              var fdn : Float = ax * b.x + ay * b.y;
              var f : Vec2 = tempVec2;
              f.x = fdn * b.x;
              f.y = fdn * b.y;
              body.applyLinearImpulse(f, p, true);
            }
          }
          ++proxy;
        }
      }
      return true;
    }
  }