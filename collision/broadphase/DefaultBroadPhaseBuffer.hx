package box2d.collision.broadphase;

import box2d.callbacks.TreeCallback;
import box2d.common.Vec2;
import box2d.callbacks.DebugDraw;
import box2d.callbacks.PairCallback;
import box2d.callbacks.TreeRayCastCallback;

import haxe.ds.Vector;

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts. This
 * broad-phase does not persist pairs. Instead, this reports potentially new pairs. It is up to the
 * client to consume the new pairs and to track subsequent overlap.
 * 
 * @author Daniel Murphy
 */
class DefaultBroadPhaseBuffer implements TreeCallback implements BroadPhase {

  public static var NULL_PROXY : Int = -1;

  private var m_tree : BroadPhaseStrategy;

  private var m_proxyCount : Int;

  private var m_moveBuffer : Vector<Int>;
  private var m_moveCapacity : Int;
  private var m_moveCount : Int;

  private var m_pairBuffer : Vector<Pair>;
  private var m_pairCapacity : Int;
  private var m_pairCount : Int;

  private var m_queryProxyId : Int;

  public function new(strategy : BroadPhaseStrategy) {
    m_proxyCount = 0;

    m_pairCapacity = 16;
    m_pairCount = 0;
    // m_pairBuffer = new Pair[m_pairCapacity];
    m_pairBuffer = new Vector<Pair>(m_pairCapacity);
    for(i in 0...m_pairCapacity) {
      m_pairBuffer[i] = new Pair();
    }

    m_moveCapacity = 16;
    m_moveCount = 0;
    // m_moveBuffer = new int[m_moveCapacity];
    m_moveBuffer = new Vector<Int>(m_moveCapacity);

    m_tree = strategy;
    m_queryProxyId = NULL_PROXY;
  }

  public function createProxy(aabb : AABB, userData : Dynamic) : Int {
    var proxyId : Int = m_tree.createProxy(aabb, userData);
    ++m_proxyCount;
    bufferMove(proxyId);
    return proxyId;
  }

  public function destroyProxy(proxyId : Int) : Void {
    unbufferMove(proxyId);
    --m_proxyCount;
    m_tree.destroyProxy(proxyId);
  }

  public function moveProxy(proxyId : Int, aabb : AABB, displacement : Vec2) : Void {
    var buffer : Bool = m_tree.moveProxy(proxyId, aabb, displacement);
    if (buffer) {
      bufferMove(proxyId);
    }
  }

  public function touchProxy(proxyId : Int) : Void {
    bufferMove(proxyId);
  }

  public function getUserData(proxyId : Int) : Dynamic {
    return m_tree.getUserData(proxyId);
  }

  public function getFatAABB(proxyId : Int) : AABB {
    return m_tree.getFatAABB(proxyId);
  }

  public function testOverlap(proxyIdA : Int, proxyIdB  : Int) : Bool {
    var a : AABB = m_tree.getFatAABB(proxyIdA);
    var b : AABB = m_tree.getFatAABB(proxyIdB);
    if (b.lowerBound.x - a.upperBound.x > 0.0 || b.lowerBound.y - a.upperBound.y > 0.0) {
      return false;
    }

    if (a.lowerBound.x - b.upperBound.x > 0.0 || a.lowerBound.y - b.upperBound.y > 0.0) {
      return false;
    }

    return true;
  }

  public function getProxyCount() : Int {
    return m_proxyCount;
  }

  public function drawTree(argDraw : DebugDraw) : Void {
    m_tree.drawTree(argDraw);
  }

  public function updatePairs(callback : PairCallback) : Void {
    // Reset pair buffer
    m_pairCount = 0;

    // Perform tree queries for all moving proxies.
    // for (int i = 0; i < m_moveCount; ++i) {
    // var i : Int = 0;
    for(i in 0 ... m_moveCount) {
    // while(i < m_moveCount) {
      m_queryProxyId = m_moveBuffer[i];
      if (m_queryProxyId == NULL_PROXY) {
        continue;
      }

      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      var fatAABB : AABB = m_tree.getFatAABB(m_queryProxyId);

      // Query tree, create pairs and add them pair buffer.
      // log.debug("quering aabb: "+m_queryProxy.aabb);
      m_tree.query(this, fatAABB);

      // ++i;
    }
    // log.debug("Number of pairs found: "+m_pairCount);

    // Reset move buffer
    m_moveCount = 0;

    // Sort the pair buffer to expose duplicates.
    // m_pairBuffer.sort(function(a : Pair, b : Pair) : Int {
    //     var result:Int = a.compareTo(b);
    //     if (a. < b) {
    //         result = -1;
    //     } else if (a > b) {
    //         result =  1;    
    //     } else {
    //         result 0;
    //     }
    //     return result;
    //     });
    // TODO: Vector sort
    // Arrays.sort(m_pairBuffer, 0, m_pairCount);

    // Send the pairs back to the client.
    var i : Int = 0;
    while (i < m_pairCount) {
      var primaryPair : Pair = m_pairBuffer[i];
      var userDataA : Dynamic = m_tree.getUserData(primaryPair.proxyIdA);
      var userDataB : Dynamic = m_tree.getUserData(primaryPair.proxyIdB);

      // log.debug("returning pair: "+userDataA+", "+userDataB);
      callback.addPair(userDataA, userDataB);
      ++i;

      // Skip any duplicate pairs.
      while (i < m_pairCount) {
        var pair : Pair = m_pairBuffer[i];
        if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB) {
          break;
        }
        ++i;
      }
    }
  }

  public function query(callback : TreeCallback, aabb : AABB) : Void {
    m_tree.query(callback, aabb);
  }

  public function raycast(callback : TreeRayCastCallback, input : RayCastInput) : Void {
    m_tree.raycast(callback, input);
  }

  public function getTreeHeight() : Int {
    return m_tree.getHeight();
  }

  public function getTreeBalance() : Int {
    return m_tree.getMaxBalance();
  }

  public function getTreeQuality() : Float {
    return m_tree.getAreaRatio();
  }

  private function bufferMove(proxyId : Int) : Void {
    if (m_moveCount == m_moveCapacity) {
      var old : Vector<Int> = m_moveBuffer;
      m_moveCapacity *= 2;
      m_moveBuffer = new Vector<Int>(m_moveCapacity);
      // TODO: array copy
      Vector.blit(old, 0, m_moveBuffer, 0, old.length);
      // System.arraycopy(old, 0, m_moveBuffer, 0, old.length);
    }

    m_moveBuffer[m_moveCount] = proxyId;
    ++m_moveCount;
  }

  private function unbufferMove(proxyId : Int) : Void {
    for(i in 0 ... m_moveCount) {
      if (m_moveBuffer[i] == proxyId) {
        m_moveBuffer[i] = NULL_PROXY;
      }
    }
  }

  /**
   * This is called from DynamicTree::query when we are gathering pairs.
   */
  public function treeCallback(proxyId : Int) : Bool {
    // A proxy cannot form a pair with itself.
    if (proxyId == m_queryProxyId) {
      return true;
    }

    // Grow the pair buffer as needed.
    if (m_pairCount == m_pairCapacity) {
      var oldBuffer : Vector<Pair> = m_pairBuffer;
      m_pairCapacity *= 2;
      m_pairBuffer = new Vector<Pair>(m_pairCapacity);
      // TODO: array copy
      Vector.blit(oldBuffer, 0, m_pairBuffer, 0, oldBuffer.length);
      // System.arraycopy(oldBuffer, 0, m_pairBuffer, 0, oldBuffer.length);
      // for (int i = oldBuffer.length; i < m_pairCapacity; i++) {
      for(i in oldBuffer.length ... m_pairCapacity) {  
        m_pairBuffer[i] = new Pair();
      }
    }

    if (proxyId < m_queryProxyId) {
      m_pairBuffer[m_pairCount].proxyIdA = proxyId;
      m_pairBuffer[m_pairCount].proxyIdB = m_queryProxyId;
    } else {
      m_pairBuffer[m_pairCount].proxyIdA = m_queryProxyId;
      m_pairBuffer[m_pairCount].proxyIdB = proxyId;
    }

    ++m_pairCount;
    return true;
  }
}