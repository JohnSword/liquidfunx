package box2d.particle.callbacks;

import box2d.common.Vec2;
import box2d.callbacks.QueryCallback;
import box2d.dynamics.Fixture;
import box2d.collision.AABB;
import box2d.collision.shapes.Shape;
import box2d.dynamics.Body;

class UpdateBodyContactsCallback implements QueryCallback {
    private var system : ParticleSystem;

    private var tempVec : Vec2 = new Vec2();

    public function reportFixture(fixture : Fixture) : Bool {
      if (fixture.isSensor()) {
        return true;
      }
      var shape : Shape = fixture.getShape();
      var b : Body = fixture.getBody();
      var bp : Vec2 = b.getWorldCenter();
      var bm : Float = b.getMass();
      var bI : Float = b.getInertia() - bm * b.getLocalCenter().lengthSquared();
      var invBm : Float = bm > 0 ? 1 / bm : 0;
      var invBI : Float = bI > 0 ? 1 / bI : 0;
      var childCount : Int = shape.getChildCount();
      for (childIndex in 0 ... childCount) {
        var aabb : AABB = fixture.getAABB(childIndex);
        var aabblowerBoundx : Float = aabb.lowerBound.x - system.m_particleDiameter;
        var aabblowerBoundy : Float = aabb.lowerBound.y - system.m_particleDiameter;
        var aabbupperBoundx : Float = aabb.upperBound.x + system.m_particleDiameter;
        var aabbupperBoundy : Float = aabb.upperBound.y + system.m_particleDiameter;
        var firstProxy : Int =
            lowerBound(
                system.m_proxyBuffer,
                system.m_proxyCount,
                computeTag(system.m_inverseDiameter * aabblowerBoundx, system.m_inverseDiameter
                    * aabblowerBoundy));
        var lastProxy : Int =
            upperBound(
                system.m_proxyBuffer,
                system.m_proxyCount,
                computeTag(system.m_inverseDiameter * aabbupperBoundx, system.m_inverseDiameter
                    * aabbupperBoundy));

        var proxy : Int = firstProxy;
        while (proxy != lastProxy) {
          var a : Int = system.m_proxyBuffer[proxy].index;
          var ap : Vec2 = system.m_positionBuffer.data[a];
          if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y
              && ap.y <= aabbupperBoundy) {
            var d : Float;
            var n : Vec2 = tempVec;
            d = fixture.computeDistance(ap, childIndex, n);
            if (d < system.m_particleDiameter) {
              var invAm : Float =
                  (system.m_flagsBuffer.data[a] & ParticleType.b2_wallParticle) != 0 ? 0 : system
                      .getParticleInvMass();
              var rpx : Float = ap.x - bp.x;
              var rpy : Float = ap.y - bp.y;
              var rpn : Float = rpx * n.y - rpy * n.x;
              if (system.m_bodyContactCount >= system.m_bodyContactCapacity) {
                var oldCapacity : Int = system.m_bodyContactCapacity;
                var newCapacity : Int =
                    system.m_bodyContactCount != 0
                        ? 2 * system.m_bodyContactCount
                        : Settings.minParticleBufferCapacity;
                system.m_bodyContactBuffer =
                    BufferUtils.reallocateBuffer(ParticleBodyContact,
                        system.m_bodyContactBuffer, oldCapacity, newCapacity);
                system.m_bodyContactCapacity = newCapacity;
              }
              var contact : ParticleBodyContact = system.m_bodyContactBuffer[system.m_bodyContactCount];
              contact.index = a;
              contact.body = b;
              contact.weight = 1 - d * system.m_inverseDiameter;
              contact.normal.x = -n.x;
              contact.normal.y = -n.y;
              contact.mass = 1 / (invAm + invBm + invBI * rpn * rpn);
              system.m_bodyContactCount++;
            }
          }
          ++proxy;
        }
      }
      return true;
    }
  }