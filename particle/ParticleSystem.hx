package box2d.particle;

import box2d.callbacks.ParticleDestructionListener;
import box2d.callbacks.ParticleQueryCallback;
import box2d.callbacks.ParticleRaycastCallback;
import box2d.collision.AABB;
import box2d.collision.shapes.Shape;
import box2d.common.BufferUtils;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import box2d.dynamics.Body;
import box2d.dynamics.TimeStep;
import box2d.dynamics.World;
import box2d.particle.callbacks.DestroyParticlesInShapeCallback;
import box2d.particle.callbacks.CreateParticleGroupCallback;
import box2d.particle.callbacks.UpdateBodyContactsCallback;
import box2d.particle.callbacks.SolveCollisionCallback;
import box2d.particle.callbacks.JoinParticleGroupsCallback;

import box2d.particle.buffers.ParticleBuffer;
import box2d.particle.buffers.ParticleBufferInt;
import box2d.particle.buffers.ParticleBufferVec2;
import box2d.particle.buffers.ParticleBufferParticleColor;

import haxe.ds.Vector;

 class ParticleSystem {
  /** All particle types that require creating pairs */
  public static var k_pairFlags : Int = ParticleType.b2_springParticle;
  /** All particle types that require creating triads */
  public static var k_triadFlags : Int = ParticleType.b2_elasticParticle;
  /** All particle types that require computing depth */
  public static var k_noPressureFlags : Int = ParticleType.b2_powderParticle;

  static var xTruncBits : Int = 12;
  static var yTruncBits : Int = 12;
  static var tagBits : Int = 8 * 4 - 1  /* sizeof(int) */;
  static var yOffset : Int = 1 << (yTruncBits - 1);
  static var yShift : Int = tagBits - yTruncBits;
  static var xShift : Int = tagBits - yTruncBits - xTruncBits;
  static var xScale : Int = 1 << xShift;
  static var xOffset : Int = xScale * (1 << (xTruncBits - 1));
  static var xMask : Int = - 1;
  static var yMask : Int = - 1;

  static public function computeTag(x : Float, y : Float) : Int {
    // TODO: check this calculation
    var calc : Float = ((Std.int(y + yOffset)) << yShift) + (((xScale * x)) + xOffset);
    return Std.int(calc);
  }

  static function computeRelativeTag(tag:Int, x:Int, y:Int) : Int {
    return tag + (y << yShift) + (x << xShift);
  }

  static function limitCapacity(capacity:Int, maxCount:Int) : Int {
    return maxCount != 0 && capacity > maxCount ? maxCount : capacity;
  }

  public var m_timestamp : Int = 0;
  public var m_allParticleFlags : Int = 0;
  public var m_allGroupFlags : Int = 0;
  public var m_density : Float = 0;
  public var m_inverseDensity : Float = 0;
  public var m_gravityScale : Float = 0;
  public var m_particleDiameter : Float = 0;
  public var m_inverseDiameter : Float = 0;
  public var m_squaredDiameter : Float = 0;

  public var m_count : Int = 0;
  public var m_internalAllocatedCapacity : Int = 0;
  public var m_maxCount : Int = 0;
  public var m_flagsBuffer : ParticleBufferInt;
  public var m_positionBuffer : ParticleBufferVec2;
  public var m_velocityBuffer : ParticleBufferVec2;
  public var m_accumulationBuffer : Vector<Float>; // temporary values
  public var m_accumulation2Buffer : Vector<Vec2>; // temporary vector values
  public var m_depthBuffer : Vector<Float>; // distance from the surface

  public var m_colorBuffer : ParticleBufferParticleColor;
  public var m_groupBuffer : Vector<ParticleGroup>;
  public var m_userDataBuffer : ParticleBuffer;

  public var m_proxyCount : Int = 0;
  public var m_proxyCapacity : Int = 0;
  public var m_proxyBuffer : Array<Proxy>;
  // public var m_proxyBuffer : Vector<Proxy>;

  public var m_contactCount : Int = 0;
  public var m_contactCapacity : Int = 0;
  public var m_contactBuffer : Vector<ParticleContact>;

  public var m_bodyContactCount : Int = 0;
  public var m_bodyContactCapacity : Int = 0;
  public var m_bodyContactBuffer : Vector<ParticleBodyContact>;

  public var m_pairCount : Int = 0;
  public var m_pairCapacity : Int = 0;
  public var m_pairBuffer : Vector<Pair>;

  public var m_triadCount : Int = 0;
  public var m_triadCapacity : Int = 0;
  public var m_triadBuffer : Vector<Triad>;

  public var m_groupCount : Int = 0;
  public var m_groupList : ParticleGroup;

  public var m_pressureStrength : Float = 0;
  public var m_dampingStrength : Float = 0;
  public var m_elasticStrength : Float = 0;
  public var m_springStrength : Float = 0;
  public var m_viscousStrength : Float = 0;
  public var m_surfaceTensionStrengthA : Float = 0;
  public var m_surfaceTensionStrengthB : Float = 0;
  public var m_powderStrength : Float = 0;
  public var m_ejectionStrength : Float = 0;
  public var m_colorMixingStrength : Float = 0;

  private var m_world : World;

  public function new(world : World) {
    m_world = world;
    m_timestamp = 0;
    m_allParticleFlags = 0;
    m_allGroupFlags = 0;
    m_density = 1;
    m_inverseDensity = 1;
    m_gravityScale = 1;
    m_particleDiameter = 1;
    m_inverseDiameter = 1;
    m_squaredDiameter = 1;

    m_count = 0;
    m_internalAllocatedCapacity = 0;
    m_maxCount = 0;

    m_proxyCount = 0;
    m_proxyCapacity = 0;

    m_contactCount = 0;
    m_contactCapacity = 0;

    m_bodyContactCount = 0;
    m_bodyContactCapacity = 0;

    m_pairCount = 0;
    m_pairCapacity = 0;

    m_triadCount = 0;
    m_triadCapacity = 0;

    m_groupCount = 0;

    m_pressureStrength = 0.05;
    m_dampingStrength = 1.0;
    m_elasticStrength = 0.25;
    m_springStrength = 0.25;
    m_viscousStrength = 0.25;
    m_surfaceTensionStrengthA = 0.1;
    m_surfaceTensionStrengthB = 0.2;
    m_powderStrength = 0.5;
    m_ejectionStrength = 0.5;
    m_colorMixingStrength = 0.5;

    m_flagsBuffer = new ParticleBufferInt();
    m_positionBuffer = new ParticleBufferVec2(Vec2);
    m_velocityBuffer = new ParticleBufferVec2(Vec2);
    m_colorBuffer = new ParticleBufferParticleColor(ParticleColor);
    m_userDataBuffer = new ParticleBuffer(cast Dynamic);

    m_proxyBuffer = new Array<Proxy>();
  }
  
//  public void assertNotSamePosition() {
//    for (int i = 0; i < m_count; i++) {
//      Vec2 vi = m_positionBuffer.data[i];
//      for (int j = i + 1; j < m_count; j++) {
//        Vec2 vj = m_positionBuffer.data[j];
//        assert(vi.x != vj.x || vi.y != vj.y);
//      }
//    }
//  }

  public function createParticle(def : ParticleDef) : Int {
    if (m_count >= m_internalAllocatedCapacity) {
      var capacity : Int = m_count != 0 ? 2 * m_count : Settings.minParticleBufferCapacity;
      capacity = limitCapacity(capacity, m_maxCount);
      capacity = limitCapacity(capacity, m_flagsBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_positionBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_velocityBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_colorBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_userDataBuffer.userSuppliedCapacity);
      if (m_internalAllocatedCapacity < capacity) {
        m_flagsBuffer.data =
            reallocateBufferInt(m_flagsBuffer, m_internalAllocatedCapacity, capacity, false);
        m_positionBuffer.data =
            reallocateVec2Buffer(m_positionBuffer, m_internalAllocatedCapacity, capacity, false);
        m_velocityBuffer.data =
            reallocateVec2Buffer(m_velocityBuffer, m_internalAllocatedCapacity, capacity, false);
        m_accumulationBuffer =
            BufferUtils.reallocateBufferFloatDeffered(m_accumulationBuffer, 0, m_internalAllocatedCapacity,
                capacity, false);
        m_accumulation2Buffer =
            BufferUtils.reallocateBufferDeffered(Vec2, m_accumulation2Buffer, 0,
                m_internalAllocatedCapacity, capacity, true);
        m_depthBuffer =
            BufferUtils.reallocateBufferFloatDeffered(m_depthBuffer, 0, m_internalAllocatedCapacity, capacity,
                true);
        m_colorBuffer.data =
            reallocateBuffer(m_colorBuffer, m_internalAllocatedCapacity, capacity, true);
        m_groupBuffer =
           cast BufferUtils.reallocateParticleGroupBufferDeffered(ParticleGroup, m_groupBuffer, 0,
                m_internalAllocatedCapacity, capacity, false);
        m_userDataBuffer.data =
            reallocateBuffer(m_userDataBuffer, m_internalAllocatedCapacity, capacity, true);
        m_internalAllocatedCapacity = capacity;
      }
    }
    if (m_count >= m_internalAllocatedCapacity) {
      return Settings.invalidParticleIndex;
    }
    var index : Int = m_count++;
    m_flagsBuffer.data[index] = def.flags;
    m_positionBuffer.data[index].setVec(def.position);
//    assertNotSamePosition();
    m_velocityBuffer.data[index].setVec(def.velocity);
    m_groupBuffer[index] = null;
    if (m_depthBuffer != null) {
      m_depthBuffer[index] = 0;
    }
    if (m_colorBuffer.data != null || def.color != null) {
      m_colorBuffer.data = requestParticleColorBuffer(ParticleColor, m_colorBuffer.data);
      m_colorBuffer.data[index].setParticleColor(def.color);
    }
    if (m_userDataBuffer.data != null || def.userData != null) {
      m_userDataBuffer.data =
          requestParticleBuffer(m_userDataBuffer.dataClass, m_userDataBuffer.data);
      m_userDataBuffer.data[index] = def.userData;
    }
    // if (m_proxyCount >= m_proxyCapacity) {
    //   var oldCapacity : Int = m_proxyCapacity;
    //   var newCapacity : Int = 49;
    //   // var newCapacity : Int = m_proxyCount != 0 ? 2 * m_proxyCount : Settings.minParticleBufferCapacity;
    //   m_proxyBuffer = BufferUtils.reallocateProxyBuffer(Proxy, m_proxyBuffer, oldCapacity, newCapacity);
    //   m_proxyCapacity = newCapacity;
    // }
    m_proxyBuffer.push(new Proxy());
    m_proxyBuffer[m_proxyCount].index = index;
    m_proxyCount++;
    return index;
  }

  public function destroyParticle(index : Int, callDestructionListener : Bool) : Void {
    var flags : Int = ParticleType.b2_zombieParticle;
    if (callDestructionListener) {
      flags |= ParticleType.b2_destructionListener;
    }
    m_flagsBuffer.data[index] |= flags;
  }

  private var temp : AABB = new AABB();
  private var dpcallback : DestroyParticlesInShapeCallback = new DestroyParticlesInShapeCallback();

  public function destroyParticlesInShape(shape : Shape, xf : Transform, callDestructionListener : Bool) : Int {
    dpcallback.init(this, shape, xf, callDestructionListener);
    shape.computeAABB(temp, xf, 0);
    m_world.queryParticleAABB2(dpcallback, temp);
    return dpcallback.destroyed;
  }

  public function destroyParticlesInGroup(group : ParticleGroup, callDestructionListener : Bool) : Void {
    for(i in group.m_firstIndex ... group.m_lastIndex) {
      destroyParticle(i, callDestructionListener);
    }
  }

  private var temp2 : AABB = new AABB();
  private var tempVec : Vec2 = new Vec2();
  private var tempTransform : Transform = new Transform();
  private var tempTransform2 : Transform = new Transform();
  private var createParticleGroupCallback : CreateParticleGroupCallback = new CreateParticleGroupCallback();
  private var tempParticleDef : ParticleDef = new ParticleDef();

  public function createParticleGroup(groupDef : ParticleGroupDef) : ParticleGroup {
    var stride : Float = getParticleStride();
    var identity : Transform = tempTransform;
    identity.setIdentity();
    var transform : Transform = tempTransform2;
    transform.setIdentity();
    var firstIndex : Int = m_count;
    if (groupDef.shape != null) {
      var particleDef : ParticleDef = tempParticleDef;
      particleDef.flags = groupDef.flags;
      particleDef.color = groupDef.color;
      particleDef.userData = groupDef.userData;
      var shape : Shape = groupDef.shape;
      transform.setVF(groupDef.position, groupDef.angle);
      var aabb : AABB = temp;
      var childCount : Int = shape.getChildCount();
      for (childIndex in 0 ... childCount) {
      // for (int childIndex = 0; childIndex < childCount; childIndex++) {
        if (childIndex == 0) {
          shape.computeAABB(aabb, identity, childIndex);
        } else {
          var childAABB : AABB = temp2;
          shape.computeAABB(childAABB, identity, childIndex);
          aabb.combineAABB(childAABB);
        }
      }
      var upperBoundY : Float = aabb.upperBound.y;
      var upperBoundX : Float = aabb.upperBound.x;
      var y : Float = MathUtils.floor(aabb.lowerBound.y / stride) * stride;
      while (y < upperBoundY) {
        var x : Float = MathUtils.floor(aabb.lowerBound.x / stride) * stride;
        while (x < upperBoundX) {
          var p : Vec2 = tempVec;
          p.x = x;
          p.y = y;
          if (shape.testPoint(identity, p)) {
            Transform.mulToOut(transform, p, p);
            particleDef.position.x = p.x;
            particleDef.position.y = p.y;
            p.subLocal(groupDef.position);
            Vec2.crossToOutUnsafeFVV(groupDef.angularVelocity, p, particleDef.velocity);
            particleDef.velocity.addLocalVec(groupDef.linearVelocity);
            createParticle(particleDef);
          }
          x += stride;
        }
        y += stride;
      }
    }
    var lastIndex : Int = m_count;

    var group : ParticleGroup = new ParticleGroup();
    group.m_system = this;
    group.m_firstIndex = firstIndex;
    group.m_lastIndex = lastIndex;
    group.m_groupFlags = groupDef.groupFlags;
    group.m_strength = groupDef.strength;
    group.m_userData = groupDef.userData;
    group.m_transform.set(transform);
    group.m_destroyAutomatically = groupDef.destroyAutomatically;
    group.m_prev = null;
    group.m_next = m_groupList;
    if (m_groupList != null) {
      m_groupList.m_prev = group;
    }
    m_groupList = group;
    ++m_groupCount;
    for(i in firstIndex ... lastIndex) {
      m_groupBuffer[i] = group;
    }

    updateContacts(true);
    if ((groupDef.flags & k_pairFlags) != 0) {
      for (k in 0 ... m_contactCount) {
        var contact : ParticleContact = m_contactBuffer[k];
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        if (a > b) {
          var temp : Int = a;
          a = b;
          b = temp;
        }
        if (firstIndex <= a && b < lastIndex) {
          if (m_pairCount >= m_pairCapacity) {
            var oldCapacity : Int = m_pairCapacity;
            var newCapacity : Int =
                m_pairCount != 0 ? 2 * m_pairCount : Settings.minParticleBufferCapacity;
            m_pairBuffer = cast BufferUtils.reallocateBuffer(Pair, m_pairBuffer, oldCapacity, newCapacity);
            m_pairCapacity = newCapacity;
          }
          var pair : Pair = m_pairBuffer[m_pairCount];
          pair.indexA = a;
          pair.indexB = b;
          pair.flags = contact.flags;
          pair.strength = groupDef.strength;
          pair.distance = MathUtils.distance(m_positionBuffer.data[a], m_positionBuffer.data[b]);
          m_pairCount++;
        }
      }
    }
    if ((groupDef.flags & k_triadFlags) != 0) {
      var diagram : VoronoiDiagram = new VoronoiDiagram(lastIndex - firstIndex);
      for(i in firstIndex ... lastIndex) {
        diagram.addGenerator(m_positionBuffer.data[i], i);
      }
      diagram.generate(stride / 2);
      createParticleGroupCallback.system = this;
      createParticleGroupCallback.def = groupDef;
      createParticleGroupCallback.firstIndex = firstIndex;
      diagram.getNodes(createParticleGroupCallback);
    }
    if ((groupDef.groupFlags & ParticleGroupType.b2_solidParticleGroup) != 0) {
      computeDepthForGroup(group);
    }

    return group;
  }

  public function joinParticleGroups(groupA : ParticleGroup, groupB : ParticleGroup) : Void {
    RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, m_count);
    RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex, groupB.m_firstIndex);

    var particleFlags : Int = 0;
    for(i in groupA.m_firstIndex ... groupB.m_lastIndex) {
      particleFlags |= m_flagsBuffer.data[i];
    }

    updateContacts(true);
    if ((particleFlags & k_pairFlags) != 0) {
      for (k in 0 ... m_contactCount) {
        var contact : ParticleContact = m_contactBuffer[k];
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        if (a > b) {
          var temp : Int = a;
          a = b;
          b = temp;
        }
        if (groupA.m_firstIndex <= a && a < groupA.m_lastIndex && groupB.m_firstIndex <= b
            && b < groupB.m_lastIndex) {
          if (m_pairCount >= m_pairCapacity) {
            var oldCapacity : Int = m_pairCapacity;
            var newCapacity : Int =
                m_pairCount != 0 ? 2 * m_pairCount : Settings.minParticleBufferCapacity;
            m_pairBuffer = cast BufferUtils.reallocateBuffer(Pair, m_pairBuffer, oldCapacity, newCapacity);
            m_pairCapacity = newCapacity;
          }
          var pair : Pair = m_pairBuffer[m_pairCount];
          pair.indexA = a;
          pair.indexB = b;
          pair.flags = contact.flags;
          pair.strength = MathUtils.min(groupA.m_strength, groupB.m_strength);
          pair.distance = MathUtils.distance(m_positionBuffer.data[a], m_positionBuffer.data[b]);
          m_pairCount++;
        }
      }
    }
    if ((particleFlags & k_triadFlags) != 0) {
      var diagram : VoronoiDiagram = new VoronoiDiagram(groupB.m_lastIndex - groupA.m_firstIndex);
      for(i in groupA.m_firstIndex ... groupB.m_lastIndex) {
        if ((m_flagsBuffer.data[i] & ParticleType.b2_zombieParticle) == 0) {
          diagram.addGenerator(m_positionBuffer.data[i], i);
        }
      }
      diagram.generate(getParticleStride() / 2);
      var callback : JoinParticleGroupsCallback = new JoinParticleGroupsCallback();
      callback.system = this;
      callback.groupA = groupA;
      callback.groupB = groupB;
      diagram.getNodes(callback);
    }

    for(i in groupB.m_firstIndex ... groupB.m_lastIndex) {
      m_groupBuffer[i] = groupA;
    }
    var groupFlags : Int = groupA.m_groupFlags | groupB.m_groupFlags;
    groupA.m_groupFlags = groupFlags;
    groupA.m_lastIndex = groupB.m_lastIndex;
    groupB.m_firstIndex = groupB.m_lastIndex;
    destroyParticleGroup(groupB);

    if ((groupFlags & ParticleGroupType.b2_solidParticleGroup) != 0) {
      computeDepthForGroup(groupA);
    }
  }

  // Only called from solveZombie() or joinParticleGroups().
  private function destroyParticleGroup(group : ParticleGroup) : Void {

    if (m_world.getParticleDestructionListener() != null) {
      m_world.getParticleDestructionListener().sayGoodbyeParticleGroup(group);
    }

    for(i in group.m_firstIndex ... group.m_lastIndex) {
      m_groupBuffer[i] = null;
    }

    if (group.m_prev != null) {
      group.m_prev.m_next = group.m_next;
    }
    if (group.m_next != null) {
      group.m_next.m_prev = group.m_prev;
    }
    if (group == m_groupList) {
      m_groupList = group.m_next;
    }

    --m_groupCount;
  }

  public function computeDepthForGroup(group : ParticleGroup) : Void {
    for(i in group.m_firstIndex ... group.m_lastIndex) {
      m_accumulationBuffer[i] = 0;
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      if (a >= group.m_firstIndex && a < group.m_lastIndex && b >= group.m_firstIndex
          && b < group.m_lastIndex) {
        var w : Float = contact.weight;
        m_accumulationBuffer[a] += w;
        m_accumulationBuffer[b] += w;
      }
    }
    m_depthBuffer = requestParticleBufferFloat(m_depthBuffer);
    for(i in group.m_firstIndex ... group.m_lastIndex) {
      var w : Float = m_accumulationBuffer[i];
      m_depthBuffer[i] = w < 0.8 ? 0 : MathUtils.MAX_VALUE;
    }
    var interationCount : Int = group.getParticleCount();
    for (t in 0 ... interationCount) {
      var updated : Bool = false;
      for (k in 0 ... m_contactCount) {
        var contact : ParticleContact = m_contactBuffer[k];
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        if (a >= group.m_firstIndex && a < group.m_lastIndex && b >= group.m_firstIndex
            && b < group.m_lastIndex) {
          var r : Float = 1 - contact.weight;
          var ap0 : Float = m_depthBuffer[a];
          var bp0 : Float = m_depthBuffer[b];
          var ap1 : Float = bp0 + r;
          var bp1 : Float = ap0 + r;
          if (ap0 > ap1) {
            m_depthBuffer[a] = ap1;
            updated = true;
          }
          if (bp0 > bp1) {
            m_depthBuffer[b] = bp1;
            updated = true;
          }
        }
      }
      if (!updated) {
        break;
      }
    }
    for(i in group.m_firstIndex ... group.m_lastIndex) {
      var p : Float = m_depthBuffer[i];
      if (p < MathUtils.MAX_VALUE) {
        m_depthBuffer[i] *= m_particleDiameter;
      } else {
        m_depthBuffer[i] = 0;
      }
    }
  }

  public function addContact(a : Int, b : Int) : Void {
    var pa : Vec2 = m_positionBuffer.data[a];
    var pb : Vec2 = m_positionBuffer.data[b];
    var dx : Float = pb.x - pa.x;
    var dy : Float = pb.y - pa.y;
    var d2 : Float = dx * dx + dy * dy;
//    assert(d2 != 0);
    if (d2 < m_squaredDiameter) {
      if (m_contactCount >= m_contactCapacity) {
        var oldCapacity : Int = m_contactCapacity;
        var newCapacity : Int =
            m_contactCount != 0 ? 2 * m_contactCount : Settings.minParticleBufferCapacity;
        m_contactBuffer =
            BufferUtils.reallocateParticleContactBuffer(ParticleContact, m_contactBuffer, oldCapacity,
                newCapacity);
        m_contactCapacity = newCapacity;
      }
      var invD : Float = d2 != 0 ? MathUtils.sqrt(1 / d2) : MathUtils.MAX_VALUE;
      var contact : ParticleContact = m_contactBuffer[m_contactCount];
      contact.indexA = a;
      contact.indexB = b;
      contact.flags = m_flagsBuffer.data[a] | m_flagsBuffer.data[b];
      contact.weight = 1 - d2 * invD * m_inverseDiameter;
      contact.normal.x = invD * dx;
      contact.normal.y = invD * dy;
      m_contactCount++;
    }
  }

  public function updateContacts(exceptZombie : Bool) : Void {
    for (p in 0 ... m_proxyCount) {
      var proxy : Proxy = m_proxyBuffer[p];
      var i : Int = proxy.index;
      var pos : Vec2 = m_positionBuffer.data[i];
      proxy.tag = computeTag(m_inverseDiameter * pos.x, m_inverseDiameter * pos.y);
    }
    // TODO: array sort
    // Arrays.sort(m_proxyBuffer, 0, m_proxyCount);
    m_proxyBuffer.sort(function (a:Proxy, b:Proxy) : Int {
      return a.compareTo(b);
    });
    m_contactCount = 0;
    var c_index : Int = 0;
    for(i in 0 ... m_proxyCount) {
      var a : Proxy = m_proxyBuffer[i];
      var rightTag : Int = computeRelativeTag(a.tag, 1, 0);
      var j : Int = i + 1;
      while ( j < m_proxyCount) {
        var b : Proxy = m_proxyBuffer[j];
        if (rightTag < b.tag) {
          break;
        }
        addContact(a.index, b.index);
        j++;
      }
      var bottomLeftTag : Int = computeRelativeTag(a.tag, -1, 1);
      while (c_index < m_proxyCount) {
        var c : Proxy = m_proxyBuffer[c_index];
        if (bottomLeftTag <= c.tag) {
          break;
        }
        c_index++;
      }
      var bottomRightTag : Int = computeRelativeTag(a.tag, 1, 1);

      for (b_index in c_index ... m_proxyCount) {
        var b : Proxy = m_proxyBuffer[b_index];
        if (bottomRightTag < b.tag) {
          break;
        }
        addContact(a.index, b.index);
      }
    }
    if (exceptZombie) {
      var j : Int = m_contactCount;
      var i : Int = 0;
      while(i < j) {
        if ((m_contactBuffer[i].flags & ParticleType.b2_zombieParticle) != 0) {
          --j;
          var temp : ParticleContact = m_contactBuffer[j];
          m_contactBuffer[j] = m_contactBuffer[i];
          m_contactBuffer[i] = temp;
          --i;
        }
        i++;
      }
      m_contactCount = j;
    }
  }

  private var ubccallback : UpdateBodyContactsCallback = new UpdateBodyContactsCallback();

  public function updateBodyContacts() : Void {
    var aabb : AABB = temp;
    aabb.lowerBound.x = MathUtils.MAX_VALUE;
    aabb.lowerBound.y = MathUtils.MAX_VALUE;
    aabb.upperBound.x = -MathUtils.MAX_VALUE;
    aabb.upperBound.y = -MathUtils.MAX_VALUE;
    for(i in 0 ... m_count) {
      var p : Vec2 = m_positionBuffer.data[i];
      Vec2.minToOut(aabb.lowerBound, p, aabb.lowerBound);
      Vec2.maxToOut(aabb.upperBound, p, aabb.upperBound);
    }
    aabb.lowerBound.x -= m_particleDiameter;
    aabb.lowerBound.y -= m_particleDiameter;
    aabb.upperBound.x += m_particleDiameter;
    aabb.upperBound.y += m_particleDiameter;
    m_bodyContactCount = 0;

    ubccallback.system = this;
    m_world.queryAABB(ubccallback, aabb);
  }

  private var sccallback : SolveCollisionCallback = new SolveCollisionCallback();

  public function solveCollision(step : TimeStep) : Void {
    var aabb : AABB = temp;
    var lowerBound : Vec2 = aabb.lowerBound;
    var upperBound : Vec2 = aabb.upperBound;
    lowerBound.x = MathUtils.MAX_VALUE;
    lowerBound.y = MathUtils.MAX_VALUE;
    upperBound.x = -MathUtils.MAX_VALUE;
    upperBound.y = -MathUtils.MAX_VALUE;
    for(i in 0 ... m_count) {
      var v : Vec2 = m_velocityBuffer.data[i];
      var p1 : Vec2 = m_positionBuffer.data[i];
      var p1x : Float = p1.x;
      var p1y : Float = p1.y;
      var p2x : Float = p1x + step.dt * v.x;
      var p2y : Float = p1y + step.dt * v.y;
      var bx : Float = p1x < p2x ? p1x : p2x;
      var by : Float = p1y < p2y ? p1y : p2y;
      lowerBound.x = lowerBound.x < bx ? lowerBound.x : bx;
      lowerBound.y = lowerBound.y < by ? lowerBound.y : by;
      var b1x : Float = p1x > p2x ? p1x : p2x;
      var b1y : Float = p1y > p2y ? p1y : p2y;
      upperBound.x = upperBound.x > b1x ? upperBound.x : b1x;
      upperBound.y = upperBound.y > b1y ? upperBound.y : b1y;
    }
    sccallback.step = step;
    sccallback.system = this;
    m_world.queryAABB(sccallback, aabb);
  }

  public function solve(step : TimeStep) : Void {
    ++m_timestamp;
    if (m_count == 0) {
      return;
    }
    m_allParticleFlags = 0;
    for(i in 0 ... m_count) {
      m_allParticleFlags |= m_flagsBuffer.data[i];
    }
    if ((m_allParticleFlags & ParticleType.b2_zombieParticle) != 0) {
      solveZombie();
    }
    if (m_count == 0) {
      return;
    }
    m_allGroupFlags = 0;
    var group : ParticleGroup = m_groupList;
    while (group != null) {
      m_allGroupFlags |= group.m_groupFlags;
      group = group.getNext();
    }
    var gravityx : Float = step.dt * m_gravityScale * m_world.getGravity().x;
    var gravityy : Float = step.dt * m_gravityScale * m_world.getGravity().y;
    var criticalVelocytySquared : Float = getCriticalVelocitySquared(step);
    for(i in 0 ... m_count) {
      var v : Vec2 = m_velocityBuffer.data[i];
      v.x += gravityx;
      v.y += gravityy;
      var v2 : Float = v.x * v.x + v.y * v.y;
      if (v2 > criticalVelocytySquared) {
        var a : Float = v2 == 0 ? MathUtils.MAX_VALUE : MathUtils.sqrt(criticalVelocytySquared / v2);
        v.x *= a;
        v.y *= a;
      }
    }
    solveCollision(step);
    if ((m_allGroupFlags & ParticleGroupType.b2_rigidParticleGroup) != 0) {
      solveRigid(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_wallParticle) != 0) {
      solveWall(step);
    }
    for(i in 0 ... m_count) {
      var pos : Vec2 = m_positionBuffer.data[i];
      var vel : Vec2 = m_velocityBuffer.data[i];
      pos.x += step.dt * vel.x;
      pos.y += step.dt * vel.y;
    }
    updateBodyContacts();
    updateContacts(false);
    if ((m_allParticleFlags & ParticleType.b2_viscousParticle) != 0) {
      solveViscous(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_powderParticle) != 0) {
      solvePowder(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_tensileParticle) != 0) {
      solveTensile(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_elasticParticle) != 0) {
      solveElastic(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_springParticle) != 0) {
      solveSpring(step);
    }
    if ((m_allGroupFlags & ParticleGroupType.b2_solidParticleGroup) != 0) {
      solveSolid(step);
    }
    if ((m_allParticleFlags & ParticleType.b2_colorMixingParticle) != 0) {
      solveColorMixing(step);
    }
    solvePressure(step);
    solveDamping(step);
  }

  private function solvePressure(step : TimeStep) : Void {
    // calculates the sum of contact-weights for each particle
    // that means dimensionless density
    for(i in 0 ... m_count) {
      m_accumulationBuffer[i] = 0;
    }
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      var a : Int = contact.index;
      var w : Float = contact.weight;
      m_accumulationBuffer[a] += w;
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      var w : Float = contact.weight;
      m_accumulationBuffer[a] += w;
      m_accumulationBuffer[b] += w;
    }
    // ignores powder particles
    if ((m_allParticleFlags & k_noPressureFlags) != 0) {
      for(i in 0 ... m_count) {
        if ((m_flagsBuffer.data[i] & k_noPressureFlags) != 0) {
          m_accumulationBuffer[i] = 0;
        }
      }
    }
    // calculates pressure as a linear function of density
    var pressurePerWeight : Float = m_pressureStrength * getCriticalPressure(step);
    for(i in 0 ... m_count) {
      var w : Float = m_accumulationBuffer[i];
      var h : Float =
          pressurePerWeight
              * MathUtils.max(0.0, MathUtils.min(w, Settings.maxParticleWeight)
                  - Settings.minParticleWeight);
      m_accumulationBuffer[i] = h;
    }
    // applies pressure between each particles in contact
    var velocityPerPressure : Float = step.dt / (m_density * m_particleDiameter);
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      var a : Int = contact.index;
      var b : Body = contact.body;
      var w : Float = contact.weight;
      var m : Float = contact.mass;
      var n : Vec2 = contact.normal;
      var p : Vec2 = m_positionBuffer.data[a];
      var h : Float = m_accumulationBuffer[a] + pressurePerWeight * w;
      var f : Vec2 = tempVec;
      var coef : Float = velocityPerPressure * w * m * h;
      f.x = coef * n.x;
      f.y = coef * n.y;
      var velData : Vec2 = m_velocityBuffer.data[a];
      var particleInvMass : Float = getParticleInvMass();
      velData.x -= particleInvMass * f.x;
      velData.y -= particleInvMass * f.y;
      b.applyLinearImpulse(f, p, true);
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      var w : Float = contact.weight;
      var n : Vec2 = contact.normal;
      var h : Float = m_accumulationBuffer[a] + m_accumulationBuffer[b];
      var fx : Float = velocityPerPressure * w * h * n.x;
      var fy : Float = velocityPerPressure * w * h * n.y;
      var velDataA : Vec2 = m_velocityBuffer.data[a];
      var velDataB : Vec2 = m_velocityBuffer.data[b];
      velDataA.x -= fx;
      velDataA.y -= fy;
      velDataB.x += fx;
      velDataB.y += fy;
    }
  }

  private function solveDamping(step : TimeStep) : Void {
    // reduces normal velocity of each contact
    var damping : Float = m_dampingStrength;
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      var a : Int = contact.index;
      var b : Body = contact.body;
      var w : Float = contact.weight;
      var m : Float = contact.mass;
      var n : Vec2 = contact.normal;
      var p : Vec2 = m_positionBuffer.data[a];
      var tempX : Float = p.x - b.m_sweep.c.x;
      var tempY : Float = p.y - b.m_sweep.c.y;
      var velA : Vec2 = m_velocityBuffer.data[a];
      // getLinearVelocityFromWorldPointToOut, with -= velA
      var vx : Float = -b.m_angularVelocity * tempY + b.m_linearVelocity.x - velA.x;
      var vy : Float = b.m_angularVelocity * tempX + b.m_linearVelocity.y - velA.y;
      // done
      var vn : Float = vx * n.x + vy * n.y;
      if (vn < 0) {
        var f : Vec2 = tempVec;
        f.x = damping * w * m * vn * n.x;
        f.y = damping * w * m * vn * n.y;
        var invMass : Float = getParticleInvMass();
        velA.x += invMass * f.x;
        velA.y += invMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, p, true);
      }
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      var w : Float = contact.weight;
      var n : Vec2 = contact.normal;
      var velA : Vec2 = m_velocityBuffer.data[a];
      var velB : Vec2 = m_velocityBuffer.data[b];
      var vx : Float = velB.x - velA.x;
      var vy : Float = velB.y - velA.y;
      var vn : Float = vx * n.x + vy * n.y;
      if (vn < 0) {
        var fx : Float = damping * w * vn * n.x;
        var fy : Float = damping * w * vn * n.y;
        velA.x += fx;
        velA.y += fy;
        velB.x -= fx;
        velB.y -= fy;
      }
    }
  }

  public function solveWall(step : TimeStep) : Void {
    for(i in 0 ... m_count) {
      if ((m_flagsBuffer.data[i] & ParticleType.b2_wallParticle) != 0) {
        var r : Vec2 = m_velocityBuffer.data[i];
        r.x = 0.0;
        r.y = 0.0;
      }
    }
  }

  private var tempVec2 : Vec2 = new Vec2();
  private var tempRot : Rot = new Rot();
  private var tempXf : Transform = new Transform();
  private var tempXf2 : Transform = new Transform();

  private function solveRigid(step : TimeStep) : Void {
    var group : ParticleGroup = m_groupList;
    while (group != null) {
      if ((group.m_groupFlags & ParticleGroupType.b2_rigidParticleGroup) != 0) {
        group.updateStatistics();
        var temp : Vec2 = tempVec;
        var cross : Vec2 = tempVec2;
        var rotation : Rot = tempRot;
        rotation.set(step.dt * group.m_angularVelocity);
        Rot.mulToOutUnsafe(rotation, group.m_center, cross);
        temp.setVec(group.m_linearVelocity).mulLocal(step.dt).addLocalVec(group.m_center).subLocal(cross);
        tempXf.p.setVec(temp);
        tempXf.q.setRot(rotation);
        Transform.mulToOut2(tempXf, group.m_transform, group.m_transform);
        var velocityTransform : Transform = tempXf2;
        velocityTransform.p.x = step.inv_dt * tempXf.p.x;
        velocityTransform.p.y = step.inv_dt * tempXf.p.y;
        velocityTransform.q.s = step.inv_dt * tempXf.q.s;
        velocityTransform.q.c = step.inv_dt * (tempXf.q.c - 1);
        for(i in group.m_firstIndex ... group.m_lastIndex) {
          Transform.mulToOutUnsafe(velocityTransform, m_positionBuffer.data[i],
              m_velocityBuffer.data[i]);
        }
      }
      group = group.getNext();
    }
  }

  private function solveElastic(step : TimeStep) : Void {
    var elasticStrength : Float = step.inv_dt * m_elasticStrength;
    for (k in 0 ... m_triadCount) {
      var triad : Triad = m_triadBuffer[k];
      if ((triad.flags & ParticleType.b2_elasticParticle) != 0) {
        var a : Int = triad.indexA;
        var b : Int = triad.indexB;
        var c : Int = triad.indexC;
        var oa : Vec2 = triad.pa;
        var ob : Vec2 = triad.pb;
        var oc : Vec2 = triad.pc;
        var pa : Vec2 = m_positionBuffer.data[a];
        var pb : Vec2 = m_positionBuffer.data[b];
        var pc : Vec2 = m_positionBuffer.data[c];
        var px : Float = 1 / 3 * (pa.x + pb.x + pc.x);
        var py : Float = 1 / 3 * (pa.y + pb.y + pc.y);
        var rs : Float = Vec2.crossVec(oa, pa) + Vec2.crossVec(ob, pb) + Vec2.crossVec(oc, pc);
        var rc : Float = Vec2.dot(oa, pa) + Vec2.dot(ob, pb) + Vec2.dot(oc, pc);
        var r2 : Float = rs * rs + rc * rc;
        var invR : Float = r2 == 0 ? MathUtils.MAX_VALUE : MathUtils.sqrt(1 / r2);
        rs *= invR;
        rc *= invR;
        var strength : Float = elasticStrength * triad.strength;
        var roax : Float = rc * oa.x - rs * oa.y;
        var roay : Float = rs * oa.x + rc * oa.y;
        var robx : Float = rc * ob.x - rs * ob.y;
        var roby : Float = rs * ob.x + rc * ob.y;
        var rocx : Float = rc * oc.x - rs * oc.y;
        var rocy : Float = rs * oc.x + rc * oc.y;
        var va : Vec2 = m_velocityBuffer.data[a];
        var vb : Vec2 = m_velocityBuffer.data[b];
        var vc : Vec2 = m_velocityBuffer.data[c];
        va.x += strength * (roax - (pa.x - px));
        va.y += strength * (roay - (pa.y - py));
        vb.x += strength * (robx - (pb.x - px));
        vb.y += strength * (roby - (pb.y - py));
        vc.x += strength * (rocx - (pc.x - px));
        vc.y += strength * (rocy - (pc.y - py));
      }
    }
  }

  private function solveSpring(step : TimeStep) : Void {
    var springStrength : Float = step.inv_dt * m_springStrength;
    for (k in 0 ... m_pairCount) {
      var pair : Pair = m_pairBuffer[k];
      if ((pair.flags & ParticleType.b2_springParticle) != 0) {
        var a : Int = pair.indexA;
        var b : Int = pair.indexB;
        var pa : Vec2 = m_positionBuffer.data[a];
        var pb : Vec2 = m_positionBuffer.data[b];
        var dx : Float = pb.x - pa.x;
        var dy : Float = pb.y - pa.y;
        var r0 : Float = pair.distance;
        var r1 : Float = MathUtils.sqrt(dx * dx + dy * dy);
        if (r1 == 0) r1 = MathUtils.MAX_VALUE;
        var strength : Float = springStrength * pair.strength;
        var fx : Float = strength * (r0 - r1) / r1 * dx;
        var fy : Float = strength * (r0 - r1) / r1 * dy;
        var va : Vec2 = m_velocityBuffer.data[a];
        var vb : Vec2 = m_velocityBuffer.data[b];
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  private function solveTensile(step : TimeStep) {
    m_accumulation2Buffer = requestParticleVec2Buffer(Vec2, m_accumulation2Buffer);
    for(i in 0 ... m_count) {
      m_accumulationBuffer[i] = 0;
      m_accumulation2Buffer[i].setZero();
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      if ((contact.flags & ParticleType.b2_tensileParticle) != 0) {
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        var w : Float = contact.weight;
        var n : Vec2 = contact.normal;
        m_accumulationBuffer[a] += w;
        m_accumulationBuffer[b] += w;
        var a2A : Vec2 = m_accumulation2Buffer[a];
        var a2B : Vec2 = m_accumulation2Buffer[b];
        var inter : Float = (1 - w) * w;
        a2A.x -= inter * n.x;
        a2A.y -= inter * n.y;
        a2B.x += inter * n.x;
        a2B.y += inter * n.y;
      }
    }
    var strengthA : Float = m_surfaceTensionStrengthA * getCriticalVelocity(step);
    var strengthB : Float = m_surfaceTensionStrengthB * getCriticalVelocity(step);
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      if ((contact.flags & ParticleType.b2_tensileParticle) != 0) {
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        var w : Float = contact.weight;
        var n : Vec2 = contact.normal;
        var a2A : Vec2 = m_accumulation2Buffer[a];
        var a2B : Vec2 = m_accumulation2Buffer[b];
        var h : Float = m_accumulationBuffer[a] + m_accumulationBuffer[b];
        var sx : Float = a2B.x - a2A.x;
        var sy : Float = a2B.y - a2A.y;
        var fn : Float = (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w;
        var fx : Float = fn * n.x;
        var fy : Float = fn * n.y;
        var va : Vec2 = m_velocityBuffer.data[a];
        var vb : Vec2 = m_velocityBuffer.data[b];
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  private function solveViscous(step : TimeStep) : Void {
    var viscousStrength : Float = m_viscousStrength;
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      var a : Int = contact.index;
      if ((m_flagsBuffer.data[a] & ParticleType.b2_viscousParticle) != 0) {
        var b : Body = contact.body;
        var w : Float = contact.weight;
        var m : Float = contact.mass;
        var p : Vec2 = m_positionBuffer.data[a];
        var va : Vec2 = m_velocityBuffer.data[a];
        var tempX : Float = p.x - b.m_sweep.c.x;
        var tempY : Float = p.y - b.m_sweep.c.y;
        var vx : Float = -b.m_angularVelocity * tempY + b.m_linearVelocity.x - va.x;
        var vy : Float = b.m_angularVelocity * tempX + b.m_linearVelocity.y - va.y;
        var f : Vec2 = tempVec;
        var pInvMass : Float = getParticleInvMass();
        f.x = viscousStrength * m * w * vx;
        f.y = viscousStrength * m * w * vy;
        va.x += pInvMass * f.x;
        va.y += pInvMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, p, true);
      }
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      if ((contact.flags & ParticleType.b2_viscousParticle) != 0) {
        var a : Int = contact.indexA;
        var b : Int = contact.indexB;
        var w : Float = contact.weight;
        var va : Vec2 = m_velocityBuffer.data[a];
        var vb : Vec2 = m_velocityBuffer.data[b];
        var vx : Float = vb.x - va.x;
        var vy : Float = vb.y - va.y;
        var fx : Float = viscousStrength * w * vx;
        var fy : Float = viscousStrength * w * vy;
        va.x += fx;
        va.y += fy;
        vb.x -= fx;
        vb.y -= fy;
      }
    }
  }

  private function solvePowder(step : TimeStep) {
    var powderStrength : Float = m_powderStrength * getCriticalVelocity(step);
    var minWeight : Float = 1.0 - Settings.particleStride;
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      var a : Int = contact.index;
      if ((m_flagsBuffer.data[a] & ParticleType.b2_powderParticle) != 0) {
        var w : Float = contact.weight;
        if (w > minWeight) {
          var b : Body = contact.body;
          var m : Float = contact.mass;
          var p : Vec2 = m_positionBuffer.data[a];
          var n : Vec2 = contact.normal;
          var f : Vec2 = tempVec;
          var va : Vec2 = m_velocityBuffer.data[a];
          var inter : Float = powderStrength * m * (w - minWeight);
          var pInvMass : Float = getParticleInvMass();
          f.x = inter * n.x;
          f.y = inter * n.y;
          va.x -= pInvMass * f.x;
          va.y -= pInvMass * f.y;
          b.applyLinearImpulse(f, p, true);
        }
      }
    }
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      if ((contact.flags & ParticleType.b2_powderParticle) != 0) {
        var w : Float = contact.weight;
        if (w > minWeight) {
          var a : Int = contact.indexA;
          var b : Int = contact.indexB;
          var n : Vec2 = contact.normal;
          var va : Vec2 = m_velocityBuffer.data[a];
          var vb : Vec2 = m_velocityBuffer.data[b];
          var inter : Float = powderStrength * (w - minWeight);
          var fx : Float = inter * n.x;
          var fy : Float = inter * n.y;
          va.x -= fx;
          va.y -= fy;
          vb.x += fx;
          vb.y += fy;
        }
      }
    }
  }

  private function solveSolid(step : TimeStep) : Void {
    // applies extra repulsive force from solid particle groups
    m_depthBuffer = requestParticleBufferFloat(m_depthBuffer);
    var ejectionStrength : Float = step.inv_dt * m_ejectionStrength;
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      if (m_groupBuffer[a] != m_groupBuffer[b]) {
        var w : Float = contact.weight;
        var n : Vec2 = contact.normal;
        var h : Float = m_depthBuffer[a] + m_depthBuffer[b];
        var va : Vec2 = m_velocityBuffer.data[a];
        var vb : Vec2 = m_velocityBuffer.data[b];
        var inter : Float = ejectionStrength * h * w;
        var fx : Float = inter * n.x;
        var fy : Float = inter * n.y;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  private function solveColorMixing(step : TimeStep) : Void {
    // mixes color between contacting particles
    m_colorBuffer.data = cast requestParticleBuffer(ParticleColor, m_colorBuffer.data);
    var colorMixing256 : Int = Std.int(256 * m_colorMixingStrength);
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      if ((m_flagsBuffer.data[a] & m_flagsBuffer.data[b] & ParticleType.b2_colorMixingParticle) != 0) {
        var colorA : ParticleColor = m_colorBuffer.data[a];
        var colorB : ParticleColor = m_colorBuffer.data[b];
        var dr : Int = (colorMixing256 * (colorB.r - colorA.r)) >> 8;
        var dg : Int = (colorMixing256 * (colorB.g - colorA.g)) >> 8;
        var db : Int = (colorMixing256 * (colorB.b - colorA.b)) >> 8;
        var da : Int = (colorMixing256 * (colorB.a - colorA.a)) >> 8;
        colorA.r += dr;
        colorA.g += dg;
        colorA.b += db;
        colorA.a += da;
        colorB.r -= dr;
        colorB.g -= dg;
        colorB.b -= db;
        colorB.a -= da;
      }
    }
  }

  private function solveZombie() : Void {
    // removes particles with zombie flag
    var newCount : Int = 0;
    var newIndices : Vector<Int> = new Vector<Int>(m_count);
    for(i in 0 ... m_count) {
      var flags : Int = m_flagsBuffer.data[i];
      if ((flags & ParticleType.b2_zombieParticle) != 0) {
        var destructionListener : ParticleDestructionListener = m_world.getParticleDestructionListener();
        if ((flags & ParticleType.b2_destructionListener) != 0 && destructionListener != null) {
          destructionListener.sayGoodbyeIndex(i);
        }
        newIndices[i] = Settings.invalidParticleIndex;
      } else {
        newIndices[i] = newCount;
        if (i != newCount) {
          m_flagsBuffer.data[newCount] = m_flagsBuffer.data[i];
          m_positionBuffer.data[newCount].setVec(m_positionBuffer.data[i]);
          m_velocityBuffer.data[newCount].setVec(m_velocityBuffer.data[i]);
          m_groupBuffer[newCount] = m_groupBuffer[i];
          if (m_depthBuffer != null) {
            m_depthBuffer[newCount] = m_depthBuffer[i];
          }
          if (m_colorBuffer.data != null) {
            m_colorBuffer.data[newCount].setParticleColor(m_colorBuffer.data[i]);
          }
          if (m_userDataBuffer.data != null) {
            m_userDataBuffer.data[newCount] = m_userDataBuffer.data[i];
          }
        }
        newCount++;
      }
    }

    // update proxies
    for (k in 0 ... m_proxyCount) {
      var proxy : Proxy = m_proxyBuffer[k];
      proxy.index = newIndices[proxy.index];
    }

    // Proxy lastProxy = std.remove_if(
    // m_proxyBuffer, m_proxyBuffer + m_proxyCount,
    // Test.IsProxyInvalid);
    // m_proxyCount = (int) (lastProxy - m_proxyBuffer);
    var j : Int = m_proxyCount;
    var i : Int = 0;
    while(i < j) {
      if (Test.IsProxyInvalid(m_proxyBuffer[i])) {
        --j;
        var temp : Proxy = m_proxyBuffer[j];
        m_proxyBuffer[j] = m_proxyBuffer[i];
        m_proxyBuffer[i] = temp;
        --i;
      }
      i++;
    }
    m_proxyCount = j;

    // update contacts
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      contact.indexA = newIndices[contact.indexA];
      contact.indexB = newIndices[contact.indexB];
    }
    // ParticleContact lastContact = std.remove_if(
    // m_contactBuffer, m_contactBuffer + m_contactCount,
    // Test.IsContactInvalid);
    // m_contactCount = (int) (lastContact - m_contactBuffer);
    j = m_contactCount;
    var i : Int = 0;
    while(i < j) {
      if (Test.IsContactInvalid(m_contactBuffer[i])) {
        --j;
        var temp : ParticleContact = m_contactBuffer[j];
        m_contactBuffer[j] = m_contactBuffer[i];
        m_contactBuffer[i] = temp;
        --i;
      }
      i++;
    }
    m_contactCount = j;

    // update particle-body contacts
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      contact.index = newIndices[contact.index];
    }
    // ParticleBodyContact lastBodyContact = std.remove_if(
    // m_bodyContactBuffer, m_bodyContactBuffer + m_bodyContactCount,
    // Test.IsBodyContactInvalid);
    // m_bodyContactCount = (int) (lastBodyContact - m_bodyContactBuffer);
    j = m_bodyContactCount;
    var i : Int = 0;
    while(i < j) {
      if (Test.IsBodyContactInvalid(m_bodyContactBuffer[i])) {
        --j;
        var temp : ParticleBodyContact = m_bodyContactBuffer[j];
        m_bodyContactBuffer[j] = m_bodyContactBuffer[i];
        m_bodyContactBuffer[i] = temp;
        --i;
      }
      i++;
    }
    m_bodyContactCount = j;

    // update pairs
    for (k in 0 ... m_pairCount) {
      var pair : Pair = m_pairBuffer[k];
      pair.indexA = newIndices[pair.indexA];
      pair.indexB = newIndices[pair.indexB];
    }
    // Pair lastPair = std.remove_if(m_pairBuffer, m_pairBuffer + m_pairCount, Test.IsPairInvalid);
    // m_pairCount = (int) (lastPair - m_pairBuffer);
    j = m_pairCount;
    var i : Int = 0;
    while(i < j) {
      if (Test.IsPairInvalid(m_pairBuffer[i])) {
        --j;
        var temp : Pair = m_pairBuffer[j];
        m_pairBuffer[j] = m_pairBuffer[i];
        m_pairBuffer[i] = temp;
        --i;
      }
      i++;
    }
    m_pairCount = j;

    // update triads
    for (k in 0 ... m_triadCount) {
      var triad : Triad = m_triadBuffer[k];
      triad.indexA = newIndices[triad.indexA];
      triad.indexB = newIndices[triad.indexB];
      triad.indexC = newIndices[triad.indexC];
    }
    // Triad lastTriad =
    // std.remove_if(m_triadBuffer, m_triadBuffer + m_triadCount, Test.isTriadInvalid);
    // m_triadCount = (int) (lastTriad - m_triadBuffer);
    j = m_triadCount;
    var i : Int = 0;
    while(i < j) {
      if (Test.IsTriadInvalid(m_triadBuffer[i])) {
        --j;
        var temp : Triad = m_triadBuffer[j];
        m_triadBuffer[j] = m_triadBuffer[i];
        m_triadBuffer[i] = temp;
        --i;
      }
      i++;
    }
    m_triadCount = j;

    // update groups
    var group : ParticleGroup = m_groupList;
    while (group != null) {
      var firstIndex : Int = newCount;
      var lastIndex : Int = 0;
      var modified : Bool = false;
      for(i in group.m_firstIndex ... group.m_lastIndex) {
        j = newIndices[i];
        if (j >= 0) {
          firstIndex = MathUtils.min(firstIndex, j);
          lastIndex = MathUtils.max(lastIndex, j + 1);
        } else {
          modified = true;
        }
      }
      if (firstIndex < lastIndex) {
        group.m_firstIndex = firstIndex;
        group.m_lastIndex = lastIndex;
        if (modified) {
          if ((group.m_groupFlags & ParticleGroupType.b2_rigidParticleGroup) != 0) {
            group.m_toBeSplit = true;
          }
        }
      } else {
        group.m_firstIndex = 0;
        group.m_lastIndex = 0;
        if (group.m_destroyAutomatically) {
          group.m_toBeDestroyed = true;
        }
      }
      group = group.getNext();
    }

    // update particle count
    m_count = newCount;
    // m_world.m_stackAllocator.Free(newIndices);

    // destroy bodies with no particles
    var group : ParticleGroup = m_groupList;
    while (group != null) {
      var next : ParticleGroup = group.getNext();
      if (group.m_toBeDestroyed) {
        destroyParticleGroup(group);
      } else if (group.m_toBeSplit) {
        // TODO: split the group
      }
      group = next;
    }
  }

  private var newIndices : NewIndices = new NewIndices();

  private function RotateBuffer(start : Int, mid : Int, end : Int) : Void {
    // move the particles assigned to the given group toward the end of array
    if (start == mid || mid == end) {
      return;
    }
    newIndices.start = start;
    newIndices.mid = mid;
    newIndices.end = end;

    BufferUtils.rotateInt(m_flagsBuffer.data, start, mid, end);
    BufferUtils.rotateDynamic(m_positionBuffer.data, start, mid, end);
    BufferUtils.rotateDynamic(m_velocityBuffer.data, start, mid, end);
    BufferUtils.rotateDynamic(m_groupBuffer, start, mid, end);
    if (m_depthBuffer != null) {
      BufferUtils.rotateFloat(m_depthBuffer, start, mid, end);
    }
    if (m_colorBuffer.data != null) {
      BufferUtils.rotateDynamic(m_colorBuffer.data, start, mid, end);
    }
    if (m_userDataBuffer.data != null) {
      BufferUtils.rotateDynamic(m_userDataBuffer.data, start, mid, end);
    }

    // update proxies
    for (k in 0 ... m_proxyCount) {
      var proxy : Proxy = m_proxyBuffer[k];
      proxy.index = newIndices.getIndex(proxy.index);
    }

    // update contacts
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      contact.indexA = newIndices.getIndex(contact.indexA);
      contact.indexB = newIndices.getIndex(contact.indexB);
    }

    // update particle-body contacts
    for (k in 0 ... m_bodyContactCount) {
      var contact : ParticleBodyContact = m_bodyContactBuffer[k];
      contact.index = newIndices.getIndex(contact.index);
    }

    // update pairs
    for (k in 0 ... m_pairCount) {
      var pair : Pair = m_pairBuffer[k];
      pair.indexA = newIndices.getIndex(pair.indexA);
      pair.indexB = newIndices.getIndex(pair.indexB);
    }

    // update triads
    for (k in 0 ... m_triadCount) {
      var triad : Triad = m_triadBuffer[k];
      triad.indexA = newIndices.getIndex(triad.indexA);
      triad.indexB = newIndices.getIndex(triad.indexB);
      triad.indexC = newIndices.getIndex(triad.indexC);
    }

    // update groups
    var group : ParticleGroup = m_groupList;
    while ( group != null) {
      group.m_firstIndex = newIndices.getIndex(group.m_firstIndex);
      group.m_lastIndex = newIndices.getIndex(group.m_lastIndex - 1) + 1;
      group = group.getNext();
    }
  }

  public function setParticleRadius(radius : Float) : Void {
    m_particleDiameter = 2 * radius;
    m_squaredDiameter = m_particleDiameter * m_particleDiameter;
    m_inverseDiameter = 1 / m_particleDiameter;
  }

  public function setParticleDensity(density : Float) : Void {
    m_density = density;
    m_inverseDensity = 1 / m_density;
  }

  public function getParticleDensity() : Float {
    return m_density;
  }

  public function setParticleGravityScale(gravityScale : Float) : Void {
    m_gravityScale = gravityScale;
  }

  public function getParticleGravityScale() : Float {
    return m_gravityScale;
  }

  public function setParticleDamping(damping : Float) : Void {
    m_dampingStrength = damping;
  }

  public function getParticleDamping() : Float {
    return m_dampingStrength;
  }

  public function getParticleRadius() : Float {
    return m_particleDiameter / 2;
  }

  private function getCriticalVelocity(step : TimeStep) : Float {
    return m_particleDiameter * step.inv_dt;
  }

  private function getCriticalVelocitySquared(step : TimeStep) : Float {
    var velocity : Float = getCriticalVelocity(step);
    return velocity * velocity;
  }

  private function getCriticalPressure(step : TimeStep) : Float {
    return m_density * getCriticalVelocitySquared(step);
  }

  private function getParticleStride() : Float {
    return Settings.particleStride * m_particleDiameter;
  }

  public function getParticleMass() : Float {
    var stride : Float = getParticleStride();
    return m_density * stride * stride;
  }

  public function getParticleInvMass() {
    return 1.777777 * m_inverseDensity * m_inverseDiameter * m_inverseDiameter;
  }

  public function getParticleFlagsBuffer() : Vector<Int> {
    return m_flagsBuffer.data;
  }

  public function getParticlePositionBuffer() : Vector<Vec2> {
    return m_positionBuffer.data;
  }

  public function getParticleVelocityBuffer() : Vector<Vec2> {
    return m_velocityBuffer.data;
  }

  public function getParticleColorBuffer() : Vector<ParticleColor> {
    m_colorBuffer.data = requestParticleColorBuffer(ParticleColor, m_colorBuffer.data);
    return m_colorBuffer.data;
  }

  public function getParticleUserDataBuffer() : Vector<Dynamic> {
    m_userDataBuffer.data = requestParticleUserDataBuffer(m_userDataBuffer.data);
    return m_userDataBuffer.data;
  }

  public function getParticleMaxCount() : Int {
    return m_maxCount;
  }

  public function setParticleMaxCount(count : Int) : Void {
    m_maxCount = count;
  }

  private function setParticleBuffer(buffer : ParticleBufferInt, newData : Vector<Int>, newCapacity : Int) : Void {
    if (buffer.userSuppliedCapacity != 0) {
      // m_world.m_blockAllocator.Free(buffer.data, sizeof(T) * m_internalAllocatedCapacity);
    }
    buffer.data = newData;
    buffer.userSuppliedCapacity = newCapacity;
  }

  private function setParticleBuffer2(buffer : Dynamic, newData : Vector<Dynamic>, newCapacity : Int) : Void {
    if (buffer.userSuppliedCapacity != 0) {
      // m_world.m_blockAllocator.Free(buffer.data, sizeof(T) * m_internalAllocatedCapacity);
    }
    buffer.data = newData;
    buffer.userSuppliedCapacity = newCapacity;
  }

  public function setParticleFlagsBuffer(buffer : Vector<Int>, capacity : Int) : Void {
    setParticleBuffer(m_flagsBuffer, buffer, capacity);
  }

  public function setParticlePositionBuffer(buffer : Vector<Vec2>, capacity : Int) : Void {
    setParticleBuffer2(m_positionBuffer, buffer, capacity);
  }

  public function setParticleVelocityBuffer(buffer : Vector<Vec2>, capacity : Int) : Void {
    setParticleBuffer2(m_velocityBuffer, buffer, capacity);
  }

  public function setParticleColorBuffer(buffer : Vector<ParticleColor>, capacity : Int) : Void {
    setParticleBuffer2(m_colorBuffer, buffer, capacity);
  }

  public function getParticleGroupBuffer() : Vector<ParticleGroup> {
    return m_groupBuffer;
  }

  public function getParticleGroupCount() : Int {
    return m_groupCount;
  }

  public function getParticleGroupList() : Vector<ParticleGroup> {
    return m_groupBuffer;
  }

  public function getParticleCount() : Int {
    return m_count;
  }

  public function setParticleUserDataBuffer(buffer : Dynamic, capacity : Int) : Void {
    setParticleBuffer2(m_userDataBuffer, buffer, capacity);
  }

  public static function lowerBound(ray : Array<Proxy>, length : Int, tag : Int) : Int {
      var left : Int = 0;
      var step : Int, curr : Int;
      while (length > 0) {
        step = Std.int(length / 2);
        curr = left + step;
        if (ray[curr].tag < tag) {
          left = curr + 1;
          length -= step + 1;
        } else {
          length = step;
        }
      }
      return left;
    }

  public static function upperBound(ray : Array<Proxy>, length : Int, tag : Int) : Int {
      var left : Int = 0;
      var step : Int, curr : Int;
      while (length > 0) {
        step = Std.int(length / 2);
        curr = left + step;
        if (ray[curr].tag <= tag) {
          left = curr + 1;
          length -= step + 1;
        } else {
          length = step;
        }
      }
      return left;
    }

  public function queryAABB(callback : ParticleQueryCallback, aabb : AABB) : Void {
    if (m_proxyCount == 0) {
      return;
    }

    var lowerBoundX : Float = aabb.lowerBound.x;
    var lowerBoundY : Float = aabb.lowerBound.y;
    var upperBoundX : Float = aabb.upperBound.x;
    var upperBoundY : Float = aabb.upperBound.y;
    var firstProxy : Int =
        lowerBound(m_proxyBuffer, m_proxyCount,
            computeTag(m_inverseDiameter * lowerBoundX, m_inverseDiameter * lowerBoundY));
    var lastProxy : Int =
        upperBound(m_proxyBuffer, m_proxyCount,
            computeTag(m_inverseDiameter * upperBoundX, m_inverseDiameter * upperBoundY));
    for (proxy in firstProxy ... lastProxy) {
    // for (int proxy = firstProxy; proxy < lastProxy; ++proxy) {
      var i : Int = m_proxyBuffer[proxy].index;
      var p : Vec2 = m_positionBuffer.data[i];
      if (lowerBoundX < p.x && p.x < upperBoundX && lowerBoundY < p.y && p.y < upperBoundY) {
        if (!callback.reportParticle(i)) {
          break;
        }
      }
    }
  }

  /**
   * @param callback
   * @param point1
   * @param point2
   */
  public function raycast(callback : ParticleRaycastCallback, point1 : Vec2, point2 : Vec2) : Void {
    if (m_proxyCount == 0) {
      return;
    }
    var firstProxy : Int =
        lowerBound(
            m_proxyBuffer,
            m_proxyCount,
            computeTag(m_inverseDiameter * MathUtils.min(point1.x, point2.x) - 1, m_inverseDiameter
                * MathUtils.min(point1.y, point2.y) - 1));
    var lastProxy : Int =
        upperBound(
            m_proxyBuffer,
            m_proxyCount,
            computeTag(m_inverseDiameter * MathUtils.max(point1.x, point2.x) + 1, m_inverseDiameter
                * MathUtils.max(point1.y, point2.y) + 1));
    var fraction : Float = 1;
    // solving the following equation:
    // ((1-t)*point1+t*point2-position)^2=diameter^2
    // where t is a potential fraction
    var vx : Float = point2.x - point1.x;
    var vy : Float = point2.y - point1.y;
    var v2 : Float = vx * vx + vy * vy;
    if (v2 == 0) v2 = MathUtils.MAX_VALUE;
    for (proxy in firstProxy ... lastProxy) {
    // for (int proxy = firstProxy; proxy < lastProxy; ++proxy) {
      var i : Int = m_proxyBuffer[proxy].index;
      var posI : Vec2 = m_positionBuffer.data[i];
      var px : Float = point1.x - posI.x;
      var py : Float = point1.y - posI.y;
      var pv : Float = px * vx + py * vy;
      var p2 : Float = px * px + py * py;
      var determinant : Float = pv * pv - v2 * (p2 - m_squaredDiameter);
      if (determinant >= 0) {
        var sqrtDeterminant : Float = MathUtils.sqrt(determinant);
        // find a solution between 0 and fraction
        var t : Float = (-pv - sqrtDeterminant) / v2;
        if (t > fraction) {
          continue;
        }
        if (t < 0) {
          t = (-pv + sqrtDeterminant) / v2;
          if (t < 0 || t > fraction) {
            continue;
          }
        }
        var n : Vec2 = tempVec;
        tempVec.x = px + t * vx;
        tempVec.y = py + t * vy;
        n.normalize();
        var point : Vec2 = tempVec2;
        point.x = point1.x + t * vx;
        point.y = point1.y + t * vy;
        var f : Float = callback.reportParticle(i, point, n, t);
        fraction = MathUtils.min(fraction, f);
        if (fraction <= 0) {
          break;
        }
      }
    }
  }

  public function computeParticleCollisionEnergy() : Float {
    var sum_v2 : Float = 0;
    for (k in 0 ... m_contactCount) {
      var contact : ParticleContact = m_contactBuffer[k];
      var a : Int = contact.indexA;
      var b : Int = contact.indexB;
      var n : Vec2 = contact.normal;
      var va : Vec2 = m_velocityBuffer.data[a];
      var vb : Vec2 = m_velocityBuffer.data[b];
      var vx : Float = vb.x - va.x;
      var vy : Float = vb.y - va.y;
      var vn : Float = vx * n.x + vy * n.y;
      if (vn < 0) {
        sum_v2 += vn * vn;
      }
    }
    return 0.5 * getParticleMass() * sum_v2;
  }

  // reallocate a buffer
  static private function reallocateBuffer(buffer : Dynamic, oldCapacity : Int, newCapacity : Int, deferred : Bool) : Dynamic {
    return BufferUtils.reallocateBufferDeffered(buffer.dataClass, buffer.data, buffer.userSuppliedCapacity,
        oldCapacity, newCapacity, deferred);
  }

  static private function reallocateVec2Buffer(buffer : ParticleBufferVec2, oldCapacity : Int, newCapacity : Int, deferred : Bool) : Dynamic {
    return BufferUtils.reallocateVec2BufferDeffered(buffer.dataClass, buffer.data, buffer.userSuppliedCapacity,
        oldCapacity, newCapacity, deferred);
  }

  static private function reallocateBufferInt(buffer : ParticleBufferInt, oldCapacity : Int, newCapacity : Int, deferred : Bool) : Vector<Int> {
    return BufferUtils.reallocateBufferIntDeffered(buffer.data, buffer.userSuppliedCapacity, oldCapacity,
        newCapacity, deferred);
  }

  private function requestParticleVec2Buffer(klass : Class<Vec2>, buffer : Vector<Vec2>) : Vector<Vec2> {
    if (buffer == null) {
      // buffer = cast Array.newInstance(klass, m_internalAllocatedCapacity);
      buffer = new Vector<Vec2>(m_internalAllocatedCapacity);
      for(i in 0 ... m_internalAllocatedCapacity) {
        try {
          buffer[i] = Type.createInstance( klass, [] );
        } catch (e : Dynamic) {
          throw Std.string(e);
        }
      }
    }
    return buffer;
  }

  private function requestParticleBuffer(klass : Class<Dynamic>, buffer : Vector<Dynamic>) : Vector<Dynamic> {
    if (buffer == null) {
      // buffer = cast Array.newInstance(klass, m_internalAllocatedCapacity);
      buffer = new Vector<Dynamic>(m_internalAllocatedCapacity);
      for(i in 0 ... m_internalAllocatedCapacity) {
        try {
          buffer[i] = Type.createInstance( klass, [] );
        } catch (e : Dynamic) {
          throw Std.string(e);
        }
      }
    }
    return buffer;
  }

  private function requestParticleColorBuffer(klass : Class<ParticleColor>, buffer : Vector<ParticleColor>) : Vector<ParticleColor> {
    if (buffer == null) {
      buffer = new Vector<ParticleColor>(m_internalAllocatedCapacity);
      for(i in 0 ... m_internalAllocatedCapacity) {
        try {
          buffer[i] = Type.createInstance( klass, [] );
        } catch (e : Dynamic) {
          throw Std.string(e);
        }
      }
    }
    return buffer;
  }

  private function requestParticleUserDataBuffer(buffer : Vector<Dynamic>) : Vector<Dynamic> {
    if (buffer == null) {
      buffer = new Vector<Dynamic>(m_internalAllocatedCapacity);
      for(i in 0 ... m_internalAllocatedCapacity) {
        try {
          buffer[i] = {};
        } catch (e : Dynamic) {
          throw Std.string(e);
        }
      }
    }
    return buffer;
  }

  private function requestParticleBufferFloat(buffer : Vector<Float>) : Vector<Float> {
    if (buffer == null) {
      buffer = new Vector<Float>(m_internalAllocatedCapacity);
    }
    return buffer;
  }
  
}


class Test {
    public static function IsProxyInvalid(proxy : Proxy) : Bool {
      return proxy.index < 0;
    }

    public static function IsContactInvalid(contact : ParticleContact) : Bool {
      return contact.indexA < 0 || contact.indexB < 0;
    }

    public static function IsBodyContactInvalid(contact : ParticleBodyContact) : Bool {
      return contact.index < 0;
    }

    public static function IsPairInvalid(pair : Pair) : Bool {
      return pair.indexA < 0 || pair.indexB < 0;
    }

    public static function IsTriadInvalid(triad : Triad) : Bool {
      return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
    }
  }

