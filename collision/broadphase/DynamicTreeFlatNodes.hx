/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package box2d.collision.broadphase;

import box2d.callbacks.DebugDraw;
import box2d.callbacks.TreeCallback;
import box2d.callbacks.TreeRayCastCallback;
import box2d.collision.AABB;
import box2d.collision.RayCastInput;
import box2d.common.BufferUtils;
import box2d.common.Color3f;
import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Vec2;

 class DynamicTreeFlatNodes implements BroadPhaseStrategy {
  public static var MAX_STACK_SIZE : Int = 64;
  public static var NULL_NODE : Int = -1;
  public static var INITIAL_BUFFER_LENGTH : Int = 16;

  public var m_root : Int;
  public var m_aabb : Array<AABB>;
  public var m_userData : Dynamic;
  public var m_parent : Int;
  public var m_child1 : Int;
  public var m_child2 : Int;
  public var m_height : Int;

  private var m_nodeCount : Int;
  private var m_nodeCapacity : Int;

  private var m_freeList : Int;

  private var drawVecs : Array<Vec2> = new Array<Vec2>(4);

  public function new() {
    m_root = NULL_NODE;
    m_nodeCount = 0;
    m_nodeCapacity = 16;
    expandBuffers(0, m_nodeCapacity);

    for(i in 0 ... drawVecs.length) {
      drawVecs[i] = new Vec2();
    }
  }

  private function expandBuffers(oldSize : Int, newSize : Int) : Void {
    m_aabb = BufferUtils.reallocateBuffer(AABB, m_aabb, oldSize, newSize);
    m_userData = BufferUtils.reallocateBuffer(Dynamic, m_userData, oldSize, newSize);
    m_parent = BufferUtils.reallocateBuffer(m_parent, oldSize, newSize);
    m_child1 = BufferUtils.reallocateBuffer(m_child1, oldSize, newSize);
    m_child2 = BufferUtils.reallocateBuffer(m_child2, oldSize, newSize);
    m_height = BufferUtils.reallocateBuffer(m_height, oldSize, newSize);

    // Build a linked list for the free list.
    for(i in oldSize ... newSize) {
      m_aabb[i] = new AABB();
      m_parent[i] = (i == newSize - 1) ? NULL_NODE : i + 1;
      m_height[i] = -1;
      m_child1[i] = -1;
      m_child2[i] = -1;
    }
    m_freeList = oldSize;
  }

  override public function createProxy(aabb : AABB, userData : Dynamic) : Int {
    var node : Int = allocateNode();
    // Fatten the aabb
    var nodeAABB : AABB = m_aabb[node];
    nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
    m_userData[node] = userData;

    insertLeaf(node);

    return node;
  }

  override public function destroyProxy(proxyId : Int) : Void {

    removeLeaf(proxyId);
    freeNode(proxyId);
  }

  override public function moveProxy(proxyId : Int, aabb : AABB, displacement : Vec2) : Bool {
    var node : Int = proxyId;

    var nodeAABB : AABB = m_aabb[node];
    if (nodeAABB.lowerBound.x <= aabb.lowerBound.x && nodeAABB.lowerBound.y <= aabb.lowerBound.y
        && aabb.upperBound.x <= nodeAABB.upperBound.x && aabb.upperBound.y <= nodeAABB.upperBound.y) {
      return false;
    }

    removeLeaf(node);

    // Extend AABB
    var lowerBound : Vec2 = nodeAABB.lowerBound;
    var upperBound : Vec2 = nodeAABB.upperBound;
    lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    upperBound.y = aabb.upperBound.y + Settings.aabbExtension;

    // Predict AABB displacement.
    var dx : Float = displacement.x * Settings.aabbMultiplier;
    var dy : Float = displacement.y * Settings.aabbMultiplier;
    if (dx < 0.0) {
      lowerBound.x += dx;
    } else {
      upperBound.x += dx;
    }

    if (dy < 0.0) {
      lowerBound.y += dy;
    } else {
      upperBound.y += dy;
    }

    insertLeaf(proxyId);
    return true;
  }

  override public function getUserData(proxyId : Int) : Dynamic {
    return m_userData[proxyId];
  }

  override public function getFatAABB(proxyId : Int) : AABB {
    return m_aabb[proxyId];
  }

  private var nodeStack : Array<Int> = new Array<Int>(20);
  private var nodeStackIndex : Int;

  override public function query(callback : TreeCallback, aabb : AABB) : Void {
    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = m_root;

    while (nodeStackIndex > 0) {
      var node : Int = nodeStack[--nodeStackIndex];
      if (node == NULL_NODE) {
        continue;
      }

      if (AABB.testOverlap(m_aabb[node], aabb)) {
        var child1 : Int = m_child1[node];
        if (child1 == NULL_NODE) {
          var proceed : Bool = callback.treeCallback(node);
          if (!proceed) {
            return;
          }
        } else {
          if (nodeStack.length - nodeStackIndex - 2 <= 0) {
            nodeStack =
                BufferUtils.reallocateBuffer(nodeStack, nodeStack.length, nodeStack.length * 2);
          }
          nodeStack[nodeStackIndex++] = child1;
          nodeStack[nodeStackIndex++] = m_child2[node];
        }
      }
    }
  }

  private var r : Vec2 = new Vec2();
  private var aabb : AABB = new AABB();
  private var subInput : RayCastInput = new RayCastInput();

  override public function raycast(callback : TreeRayCastCallback, input : RayCastInput) : Void {
    var p1 : Vec2 = input.p1;
    var p2 : Vec2 = input.p2;
    var p1x : Float = p1.x, p2x = p2.x, p1y = p1.y, p2y = p2.y;
    var vx : Float, vy : Float;
    var rx : Float, ry : Float;
    var absVx : Float, absVy : Float;
    var cx : Float, cy : Float;
    var hx : Float, hy : Float;
    var tempx : Float, tempy : Float;
    r.x = p2x - p1x;
    r.y = p2y - p1y;
    r.normalize();
    rx = r.x;
    ry = r.y;

    // v is perpendicular to the segment.
    vx = -1 * ry;
    vy = 1 * rx;
    absVx = MathUtils.abs(vx);
    absVy = MathUtils.abs(vy);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    var maxFraction : Float = input.maxFraction;

    // Build a bounding box for the segment.
    var segAABB : AABB = aabb;
    // Vec2 t = p1 + maxFraction * (p2 - p1);
    // before inline
    // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
    // Vec2.minToOut(p1, temp, segAABB.lowerBound);
    // Vec2.maxToOut(p1, temp, segAABB.upperBound);
    tempx = (p2x - p1x) * maxFraction + p1x;
    tempy = (p2y - p1y) * maxFraction + p1y;
    segAABB.lowerBound.x = p1x < tempx ? p1x : tempx;
    segAABB.lowerBound.y = p1y < tempy ? p1y : tempy;
    segAABB.upperBound.x = p1x > tempx ? p1x : tempx;
    segAABB.upperBound.y = p1y > tempy ? p1y : tempy;
    // end inline

    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = m_root;
    while (nodeStackIndex > 0) {
      var node : Int = nodeStack[--nodeStackIndex] = m_root;
      if (node == NULL_NODE) {
        continue;
      }

      var nodeAABB : AABB = m_aabb[node];
      if (!AABB.testOverlap(nodeAABB, segAABB)) {
        continue;
      }

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)
      // node.aabb.getCenterToOut(c);
      // node.aabb.getExtentsToOut(h);
      cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5f;
      cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5f;
      hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5f;
      hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5f;
      tempx = p1x - cx;
      tempy = p1y - cy;
      var separation : Float = MathUtils.abs(vx * tempx + vy * tempy) - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      var child1 : Int = m_child1[node];
      if (child1 == NULL_NODE) {
        subInput.p1.x = p1x;
        subInput.p1.y = p1y;
        subInput.p2.x = p2x;
        subInput.p2.y = p2y;
        subInput.maxFraction = maxFraction;

        var value : Float = callback.raycastCallback(subInput, node);

        if (value == 0.0) {
          // The client has terminated the ray cast.
          return;
        }

        if (value > 0.0) {
          // Update segment bounding box.
          maxFraction = value;
          // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
          // Vec2.minToOut(p1, temp, segAABB.lowerBound);
          // Vec2.maxToOut(p1, temp, segAABB.upperBound);
          tempx = (p2x - p1x) * maxFraction + p1x;
          tempy = (p2y - p1y) * maxFraction + p1y;
          segAABB.lowerBound.x = p1x < tempx ? p1x : tempx;
          segAABB.lowerBound.y = p1y < tempy ? p1y : tempy;
          segAABB.upperBound.x = p1x > tempx ? p1x : tempx;
          segAABB.upperBound.y = p1y > tempy ? p1y : tempy;
        }
      } else {
        nodeStack[nodeStackIndex++] = child1;
        nodeStack[nodeStackIndex++] = m_child2[node];
      }
    }
  }

  override public function computeHeight() : Int {
    return computeHeight(m_root);
  }

  private function computeHeight(node : Int) : Int {

    if (m_child1[node] == NULL_NODE) {
      return 0;
    }
    var height1 : Int = computeHeight(m_child1[node]);
    var height2 : Int = computeHeight(m_child2[node]);
    return 1 + MathUtils.max(height1, height2);
  }

  /**
   * Validate this tree. For testing.
   */
  public function validate() : Void {
    validateStructure(m_root);
    validateMetrics(m_root);

    var freeCount : Int = 0;
    var freeNode : Int = m_freeList;
    while (freeNode != NULL_NODE) {
      freeNode = m_parent[freeNode];
      ++freeCount;
    }

  }

  override public function getHeight() : Int {
    if (m_root == NULL_NODE) {
      return 0;
    }
    return m_height[m_root];
  }

  override public function getMaxBalance() : Int {
    var maxBalance : Int = 0;
    var i : Int = 0;
    while (i < m_nodeCapacity) {
    // for (int i = 0; i < m_nodeCapacity; ++i) {
      if (m_height[i] <= 1) {
        continue;
      }


      var child1 : Int = m_child1[i];
      var child2 : Int = m_child2[i];
      var balance : Int = MathUtils.abs(m_height[child2] - m_height[child1]);
      maxBalance = MathUtils.max(maxBalance, balance);

      ++i;
    }

    return maxBalance;
  }

  override public function getAreaRatio() : Float {
    if (m_root == NULL_NODE) {
      return 0.0;
    }

    var root : Int = m_root;
    var rootArea : Float = m_aabb[root].getPerimeter();

    var totalArea : Float = 0.0;
    var i : Int = 0;
    while (i < m_nodeCapacity) {
    // for (int i = 0; i < m_nodeCapacity; ++i) {
      if (m_height[i] < 0) {
        // Free node in pool
        continue;
      }

      totalArea += m_aabb[i].getPerimeter();
      ++i;
    }

    return totalArea / rootArea;
  }

  // /**
  // * Build an optimal tree. Very expensive. For testing.
  // */
  // public void rebuildBottomUp() {
  // int[] nodes = new int[m_nodeCount];
  // int count = 0;
  //
  // // Build array of leaves. Free the rest.
  // for (int i = 0; i < m_nodeCapacity; ++i) {
  // if (m_nodes[i].height < 0) {
  // // free node in pool
  // continue;
  // }
  //
  // DynamicTreeNode node = m_nodes[i];
  // if (node.isLeaf()) {
  // node.parent = null;
  // nodes[count] = i;
  // ++count;
  // } else {
  // freeNode(node);
  // }
  // }
  //
  // AABB b = new AABB();
  // while (count > 1) {
  // float minCost = Float.MAX_VALUE;
  // int iMin = -1, jMin = -1;
  // for (int i = 0; i < count; ++i) {
  // AABB aabbi = m_nodes[nodes[i]].aabb;
  //
  // for (int j = i + 1; j < count; ++j) {
  // AABB aabbj = m_nodes[nodes[j]].aabb;
  // b.combine(aabbi, aabbj);
  // float cost = b.getPerimeter();
  // if (cost < minCost) {
  // iMin = i;
  // jMin = j;
  // minCost = cost;
  // }
  // }
  // }
  //
  // int index1 = nodes[iMin];
  // int index2 = nodes[jMin];
  // DynamicTreeNode child1 = m_nodes[index1];
  // DynamicTreeNode child2 = m_nodes[index2];
  //
  // DynamicTreeNode parent = allocateNode();
  // parent.child1 = child1;
  // parent.child2 = child2;
  // parent.height = 1 + MathUtils.max(child1.height, child2.height);
  // parent.aabb.combine(child1.aabb, child2.aabb);
  // parent.parent = null;
  //
  // child1.parent = parent;
  // child2.parent = parent;
  //
  // nodes[jMin] = nodes[count - 1];
  // nodes[iMin] = parent.id;
  // --count;
  // }
  //
  // m_root = m_nodes[nodes[0]];
  //
  // validate();
  // }

  private function allocateNode() : Int {
    if (m_freeList == NULL_NODE) {
      m_nodeCapacity *= 2;
      expandBuffers(m_nodeCount, m_nodeCapacity);
    }
    var node : Int = m_freeList;
    m_freeList = m_parent[node];
    m_parent[node] = NULL_NODE;
    m_child1[node] = NULL_NODE;
    m_height[node] = 0;
    ++m_nodeCount;
    return node;
  }

  /**
   * returns a node to the pool
   */
  private function freeNode(node : Int) : Void {
    m_parent[node] = m_freeList != NULL_NODE ? m_freeList : NULL_NODE;
    m_height[node] = -1;
    m_freeList = node;
    m_nodeCount--;
  }

  private var combinedAABB : AABB = new AABB();

  private function insertLeaf(leaf : Int) : Void {
    if (m_root == NULL_NODE) {
      m_root = leaf;
      m_parent[m_root] = NULL_NODE;
      return;
    }

    // find the best sibling
    var leafAABB : AABB = m_aabb[leaf];
    var index : Int = m_root;
    while (m_child1[index] != NULL_NODE) {
      var node : Int = index;
      var child1 : Int = m_child1[node];
      var child2 : Int = m_child2[node];
      var nodeAABB : AABB = m_aabb[node];
      var area : Float = nodeAABB.getPerimeter();

      combinedAABB.combine(nodeAABB, leafAABB);
      var combinedArea : Float = combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      var cost : Float = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      var inheritanceCost : Float = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      var cost1 : Float;
      var child1AABB : AABB = m_aabb[child1];
      if (m_child1[child1] == NULL_NODE) {
        combinedAABB.combine(leafAABB, child1AABB);
        cost1 = combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        combinedAABB.combine(leafAABB, child1AABB);
        var oldArea : Float = child1AABB.getPerimeter();
        var newArea : Float = combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      var cost2 : Float;
      var child2AABB : AABB = m_aabb[child2];
      if (m_child1[child2] == NULL_NODE) {
        combinedAABB.combine(leafAABB, child2AABB);
        cost2 = combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        combinedAABB.combine(leafAABB, child2AABB);
        var oldArea : Float = child2AABB.getPerimeter();
        var newArea : Float = combinedAABB.getPerimeter();
        cost2 = newArea - oldArea + inheritanceCost;
      }

      // Descend according to the minimum cost.
      if (cost < cost1 && cost < cost2) {
        break;
      }

      // Descend
      if (cost1 < cost2) {
        index = child1;
      } else {
        index = child2;
      }
    }

    var sibling : Int = index;
    var oldParent : Int = m_parent[sibling];
    var newParent : Int = allocateNode();
    m_parent[newParent] = oldParent;
    m_userData[newParent] = null;
    m_aabb[newParent].combine(leafAABB, m_aabb[sibling]);
    m_height[newParent] = m_height[sibling] + 1;

    if (oldParent != NULL_NODE) {
      // The sibling was not the root.
      if (m_child1[oldParent] == sibling) {
        m_child1[oldParent] = newParent;
      } else {
        m_child2[oldParent] = newParent;
      }

      m_child1[newParent] = sibling;
      m_child2[newParent] = leaf;
      m_parent[sibling] = newParent;
      m_parent[leaf] = newParent;
    } else {
      // The sibling was the root.
      m_child1[newParent] = sibling;
      m_child2[newParent] = leaf;
      m_parent[sibling] = newParent;
      m_parent[leaf] = newParent;
      m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = m_parent[leaf];
    while (index != NULL_NODE) {
      index = balance(index);

      var child1 : Int = m_child1[index];
      var child2 : Int = m_child2[index];


      m_height[index] = 1 + MathUtils.max(m_height[child1], m_height[child2]);
      m_aabb[index].combine(m_aabb[child1], m_aabb[child2]);

      index = m_parent[index];
    }
    // validate();
  }

  private function removeLeaf(leaf : Int) : Void {
    if (leaf == m_root) {
      m_root = NULL_NODE;
      return;
    }

    var parent : Int = m_parent[leaf];
    var grandParent : Int = m_parent[parent];
    var parentChild1 : Int = m_child1[parent];
    var parentChild2 : Int = m_child2[parent];
    var sibling : Int;
    if (parentChild1 == leaf) {
      sibling = parentChild2;
    } else {
      sibling = parentChild1;
    }

    if (grandParent != NULL_NODE) {
      // Destroy parent and connect sibling to grandParent.
      if (m_child1[grandParent] == parent) {
        m_child1[grandParent] = sibling;
      } else {
        m_child2[grandParent] = sibling;
      }
      m_parent[sibling] = grandParent;
      freeNode(parent);

      // Adjust ancestor bounds.
      var index : Int = grandParent;
      while (index != NULL_NODE) {
        index = balance(index);

        var child1 : Int = m_child1[index];
        var child2 : Int = m_child2[index];

        m_aabb[index].combine(m_aabb[child1], m_aabb[child2]);
        m_height[index] = 1 + MathUtils.max(m_height[child1], m_height[child2]);

        index = m_parent[index];
      }
    } else {
      m_root = sibling;
      m_parent[sibling] = NULL_NODE;
      freeNode(parent);
    }

    // validate();
  }

  // Perform a left or right rotation if node A is imbalanced.
  // Returns the new root index.
  private function balance(iA : Int) : Int {

    var A : Int = iA;
    if (m_child1[A] == NULL_NODE || m_height[A] < 2) {
      return iA;
    }

    var iB : Int = m_child1[A];
    var iC : Int = m_child2[A];

    var B : Int = iB;
    var C : Int = iC;

    var balance : Int = m_height[C] - m_height[B];

    // Rotate C up
    if (balance > 1) {
      var iF : Int = m_child1[C];
      var iG : Int = m_child2[C];
      var F : Int = iF;
      var G : Int = iG;

      // Swap A and C
      m_child1[C] = iA;
      var cParent : Int = m_parent[C] = m_parent[A];
      m_parent[A] = iC;

      // A's old parent should point to C
      if (cParent != NULL_NODE) {
        if (m_child1[cParent] == iA) {
          m_child1[cParent] = iC;
        } else {
          m_child2[cParent] = iC;
        }
      } else {
        m_root = iC;
      }

      // Rotate
      if (m_height[F] > m_height[G]) {
        m_child2[C] = iF;
        m_child2[A] = iG;
        m_parent[G] = iA;
        m_aabb[A].combine(m_aabb[B], m_aabb[G]);
        m_aabb[C].combine(m_aabb[A], m_aabb[F]);

        m_height[A] = 1 + MathUtils.max(m_height[B], m_height[G]);
        m_height[C] = 1 + MathUtils.max(m_height[A], m_height[F]);
      } else {
        m_child2[C] = iG;
        m_child2[A] = iF;
        m_parent[F] = iA;
        m_aabb[A].combine(m_aabb[B], m_aabb[F]);
        m_aabb[C].combine(m_aabb[A], m_aabb[G]);

        m_height[A] = 1 + MathUtils.max(m_height[B], m_height[F]);
        m_height[C] = 1 + MathUtils.max(m_height[A], m_height[G]);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      var iD : Int = m_child1[B];
      var iE : Int = m_child2[B];
      var D : Int = iD;
      var E : Int = iE;

      // Swap A and B
      m_child1[B] = iA;
      var Bparent : Int = m_parent[B] = m_parent[A];
      m_parent[A] = iB;

      // A's old parent should point to B
      if (Bparent != NULL_NODE) {
        if (m_child1[Bparent] == iA) {
          m_child1[Bparent] = iB;
        } else {
          m_child2[Bparent] = iB;
        }
      } else {
        m_root = iB;
      }

      // Rotate
      if (m_height[D] > m_height[E]) {
        m_child2[B] = iD;
        m_child1[A] = iE;
        m_parent[E] = iA;
        m_aabb[A].combine(m_aabb[C], m_aabb[E]);
        m_aabb[B].combine(m_aabb[A], m_aabb[D]);

        m_height[A] = 1 + MathUtils.max(m_height[C], m_height[E]);
        m_height[B] = 1 + MathUtils.max(m_height[A], m_height[D]);
      } else {
        m_child2[B] = iE;
        m_child1[A] = iD;
        m_parent[D] = iA;
        m_aabb[A].combine(m_aabb[C], m_aabb[D]);
        m_aabb[B].combine(m_aabb[A], m_aabb[E]);

        m_height[A] = 1 + MathUtils.max(m_height[C], m_height[D]);
        m_height[B] = 1 + MathUtils.max(m_height[A], m_height[E]);
      }

      return iB;
    }

    return iA;
  }

  private function validateStructure(node : Int) : Void {
    if (node == NULL_NODE) {
      return;
    }

    if (node == m_root) {
    }

    var child1 : Int = m_child1[node];
    var child2 : Int = m_child2[node];

    if (child1 == NULL_NODE) {
      return;
    }



    validateStructure(child1);
    validateStructure(child2);
  }

  private function validateMetrics(node : Int) : Void {
    if (node == NULL_NODE) {
      return;
    }

    var child1 : Int = m_child1[node];
    var child2 : Int = m_child2[node];

    if (child1 == NULL_NODE) {
      return;
    }


    var height1 : Int = m_height[child1];
    var height2 : Int = m_height[child2];
    int height;
    height = 1 + MathUtils.max(height1, height2);

    var aabb : AABB = new AABB();
    aabb.combine(m_aabb[child1], m_aabb[child2]);


    validateMetrics(child1);
    validateMetrics(child2);
  }

  override public function drawTree(argDraw : DebugDraw) : Void {
    if (m_root == NULL_NODE) {
      return;
    }
    var height : Int = computeHeight();
    drawTree(argDraw, m_root, 0, height);
  }

  private var color : Color3f = new Color3f();
  private var textVec : Vec2 = new Vec2();

  public function drawTree(argDraw : DebugDraw, node : Int, spot : Int, height : Int) : Void {
    var a : AABB = m_aabb[node];
    a.getVertices(drawVecs);

    color.set(1, (height - spot) * 1 / height, (height - spot) * 1 / height);
    argDraw.drawPolygon(drawVecs, 4, color);

    argDraw.getViewportTranform().getWorldToScreen(a.upperBound, textVec);
    argDraw.drawString(textVec.x, textVec.y, node + "-" + (spot + 1) + "/" + height, color);

    var c1 : Int = m_child1[node];
    var c2 : Int = m_child2[node];
    if (c1 != NULL_NODE) {
      drawTree(argDraw, c1, spot + 1, height);
    }
    if (c2 != NULL_NODE) {
      drawTree(argDraw, c2, spot + 1, height);
    }
  }
}

