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
import box2d.common.Color3f;
import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Vec2;

import haxe.ds.Vector;

/**
 * A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and
 * ray casts. Leafs are proxies with an AABB. In the tree we expand the proxy AABB by _fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client object to move by
 * small amounts without triggering a tree update.
 * 
 * @author daniel
 */
 class DynamicTree implements BroadPhaseStrategy {
  public static var MAX_STACK_SIZE : Int = 64;
  public static var NULL_NODE : Int = -1;

  private var m_root : DynamicTreeNode;
  private var m_nodes : Vector<DynamicTreeNode>;
  private var m_nodeCount : Int;
  private var m_nodeCapacity : Int;

  private var m_freeList : Int;

  private var drawVecs : Vector<Vec2> = new Vector<Vec2>(4);
  private var nodeStack : Vector<DynamicTreeNode> = new Vector<DynamicTreeNode>(20);
  private var nodeStackIndex : Int = 0;

  public function new() {
    m_root = null;
    m_nodeCount = 0;
    m_nodeCapacity = 16;
    m_nodes = new Vector<DynamicTreeNode>(16);

    // Build a linked list for the free list.
    var i : Int = m_nodeCapacity;
    while(i >= 0) {
    // for (int i = m_nodeCapacity - 1; i >= 0; i--) {
      m_nodes[i] = new DynamicTreeNode(i);
      m_nodes[i].parent = (i == m_nodeCapacity - 1) ? null : m_nodes[i + 1];
      m_nodes[i].height = -1;
      i--;
    }
    m_freeList = 0;

    for(i in 0 ... drawVecs.length) {
      drawVecs[i] = new Vec2();
    }
  }

  public function createProxy(aabb : AABB, userData : Dynamic) : Int {
    var node : DynamicTreeNode = allocateNode();
    var proxyId : Int = node.id;
    // Fatten the aabb
    var nodeAABB : AABB = node.aabb;
    nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
    node.userData = userData;

    insertLeaf(proxyId);

    return proxyId;
  }

  public function destroyProxy(proxyId : Int) : Void {
    var node : DynamicTreeNode = m_nodes[proxyId];

    removeLeaf(node);
    freeNode(node);
  }

  public function moveProxy(proxyId : Int, aabb : AABB, displacement : Vec2) : Bool {
    var node : DynamicTreeNode = m_nodes[proxyId];

    var nodeAABB : AABB = node.aabb;
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

  public function getUserData(proxyId : Int) : Dynamic {
    return m_nodes[proxyId].userData;
  }

  public function getFatAABB(proxyId : Int) : AABB {
    return m_nodes[proxyId].aabb;
  }

  public function query(callback : TreeCallback, aabb : AABB) : Void {
    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = m_root;

    while (nodeStackIndex > 0) {
      var node : DynamicTreeNode = nodeStack[--nodeStackIndex];
      if (node == null) {
        continue;
      }

      if (AABB.testOverlap(node.aabb, aabb)) {
        if (node.child1 == null) {
          var proceed : Bool = callback.treeCallback(node.id);
          if (!proceed) {
            return;
          }
        } else {
          if (nodeStack.length - nodeStackIndex - 2 <= 0) {
            var newBuffer : Vector<DynamicTreeNode> = new Vector<DynamicTreeNode>(nodeStack.length * 2);
            // TODO: array copy
            Vector.blit(nodeStack, 0, newBuffer, 0, nodeStack.length);
            // System.arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.length);
            nodeStack = newBuffer;
          }
          nodeStack[nodeStackIndex++] = node.child1;
          nodeStack[nodeStackIndex++] = node.child2;
        }
      }
    }
  }

  private var r : Vec2 = new Vec2();
  private var aabb : AABB = new AABB();
  private var subInput : RayCastInput = new RayCastInput();

  public function raycast(callback : TreeRayCastCallback, input : RayCastInput) : Void {
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
      var node : DynamicTreeNode = nodeStack[--nodeStackIndex];
      if (node == null) {
        continue;
      }

      var nodeAABB : AABB = node.aabb;
      if (!AABB.testOverlap(nodeAABB, segAABB)) {
        continue;
      }

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)
      // node.aabb.getCenterToOut(c);
      // node.aabb.getExtentsToOut(h);
      cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5;
      cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5;
      hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5;
      hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5;
      tempx = p1x - cx;
      tempy = p1y - cy;
      var separation : Float = MathUtils.abs(vx * tempx + vy * tempy) - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      if (node.child1 == null) {
        subInput.p1.x = p1x;
        subInput.p1.y = p1y;
        subInput.p2.x = p2x;
        subInput.p2.y = p2y;
        subInput.maxFraction = maxFraction;

        var value : Float = callback.raycastCallback(subInput, node.id);

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
        if (nodeStack.length - nodeStackIndex - 2 <= 0) {
          var newBuffer : Vector<DynamicTreeNode> = new Vector<DynamicTreeNode>(nodeStack.length * 2);
          // TODO: array copy
          Vector.blit(nodeStack, 0, newBuffer, 0, nodeStack.length);
          // System.arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.length);
          nodeStack = newBuffer;
        }
        nodeStack[nodeStackIndex++] = node.child1;
        nodeStack[nodeStackIndex++] = node.child2;
      }
    }
  }

  public function computeHeight() : Int {
    return computeHeightNode(m_root);
  }

  private function computeHeightNode(node : DynamicTreeNode) : Int {

    if (node.child1 == null) {
      return 0;
    }
    
    var height1 : Int = computeHeightNode(node.child1);
    var height2 : Int = computeHeightNode(node.child2);
    return 1 + MathUtils.max(height1, height2);
  }

  /**
   * Validate this tree. For testing.
   */
  public function validate() : Void {
    validateStructure(m_root);
    validateMetrics(m_root);

    var freeCount : Int = 0;
    var freeNode : DynamicTreeNode = m_freeList != NULL_NODE ? m_nodes[m_freeList] : null;
    while (freeNode != null) {
      freeNode = freeNode.parent;
      ++freeCount;
    }


  }

  public function getHeight() : Int {
    if (m_root == null) {
      return 0;
    }
    return m_root.height;
  }

  public function getMaxBalance() : Int {
    var maxBalance : Int = 0;
    var i : Int = 0;
    while(i < m_nodeCapacity) {
    // for (int i = 0; i < m_nodeCapacity; ++i) {
      var node : DynamicTreeNode = m_nodes[i];
      if (node.height <= 1) {
        continue;
      }
      var child1 : DynamicTreeNode = node.child1;
      var child2 : DynamicTreeNode = node.child2;
      var balance : Int = MathUtils.abs(child2.height - child1.height);
      maxBalance = MathUtils.max(maxBalance, balance);
      ++i;
    }

    return maxBalance;
  }

  public function getAreaRatio() : Float {
    if (m_root == null) {
      return 0.0;
    }

    var root : DynamicTreeNode = m_root;
    var rootArea : Float = root.aabb.getPerimeter();

    var totalArea : Float = 0.0;
    var i : Int = 0;
    while(i < m_nodeCapacity) {
    // for (int i = 0; i < m_nodeCapacity; ++i) {
      var node : DynamicTreeNode = m_nodes[i];
      if (node.height < 0) {
        // Free node in pool
        continue;
      }

      totalArea += node.aabb.getPerimeter();
      ++i;
    }

    return totalArea / rootArea;
  }

  /**
   * Build an optimal tree. Very expensive. For testing.
   */
  public function rebuildBottomUp() : Void {
    var nodes : Vector<Int> = new Vector<Int>(m_nodeCount);
    var count : Int = 0;

    // Build array of leaves. Free the rest.
    var i : Int = 0;
    while(i < m_nodeCapacity) {
    // for (int i = 0; i < m_nodeCapacity; ++i) {
      if (m_nodes[i].height < 0) {
        // free node in pool
        continue;
      }

      var node : DynamicTreeNode = m_nodes[i];
      if (node.child1 == null) {
        node.parent = null;
        nodes[count] = i;
        ++count;
      } else {
        freeNode(node);
      }
      ++i;
    }

    var b : AABB = new AABB();
    while (count > 1) {
      var minCost : Float = MathUtils.MAX_VALUE;
      var iMin : Int = -1, jMin = -1;
      var i : Int = 0;
      while (i < count) {
      // for (int i = 0; i < count; ++i) {
        var aabbi : AABB = m_nodes[nodes[i]].aabb;
        var j : Int = i + 1;
        while (j < count) {
        // for (int j = i + 1; j < count; ++j) {
          var aabbj : AABB = m_nodes[nodes[j]].aabb;
          b.combine(aabbi, aabbj);
          var cost : Float = b.getPerimeter();
          if (cost < minCost) {
            iMin = i;
            jMin = j;
            minCost = cost;
          }
          ++j;
        }
        ++i;
      }

      var index1 : Int = nodes[iMin];
      var index2 : Int = nodes[jMin];
      var child1 : DynamicTreeNode = m_nodes[index1];
      var child2 : DynamicTreeNode = m_nodes[index2];

      var parent : DynamicTreeNode = allocateNode();
      parent.child1 = child1;
      parent.child2 = child2;
      parent.height = 1 + MathUtils.max(child1.height, child2.height);
      parent.aabb.combine(child1.aabb, child2.aabb);
      parent.parent = null;

      child1.parent = parent;
      child2.parent = parent;

      nodes[jMin] = nodes[count - 1];
      nodes[iMin] = parent.id;
      --count;
    }

    m_root = m_nodes[nodes[0]];

    validate();
  }

  private function allocateNode() : DynamicTreeNode {
    if (m_freeList == NULL_NODE) {

      var old : Vector<DynamicTreeNode> = m_nodes;
      m_nodeCapacity *= 2;
      m_nodes = new Vector<DynamicTreeNode>(m_nodeCapacity);
      // TODO: Array copy
      Vector.blit(old, 0, m_nodes, 0, old.length);
      // System.arraycopy(old, 0, m_nodes, 0, old.length);

      // Build a linked list for the free list.
      var i : Int = m_nodeCapacity - 1;
      while (i >= m_nodeCount) {
      // for (int i = m_nodeCapacity - 1; i >= m_nodeCount; i--) {
        m_nodes[i] = new DynamicTreeNode(i);
        m_nodes[i].parent = (i == m_nodeCapacity - 1) ? null : m_nodes[i + 1];
        m_nodes[i].height = -1;
        i--;
      }
      m_freeList = m_nodeCount;
    }
    var nodeId : Int = m_freeList;
    var treeNode : DynamicTreeNode = m_nodes[nodeId];
    m_freeList = treeNode.parent != null ? treeNode.parent.id : NULL_NODE;

    treeNode.parent = null;
    treeNode.child1 = null;
    treeNode.child2 = null;
    treeNode.height = 0;
    treeNode.userData = null;
    ++m_nodeCount;
    return treeNode;
  }

  /**
   * returns a node to the pool
   */
  private function freeNode(node : DynamicTreeNode) : Void {
    node.parent = m_freeList != NULL_NODE ? m_nodes[m_freeList] : null;
    node.height = -1;
    m_freeList = node.id;
    m_nodeCount--;
  }

  private var combinedAABB : AABB = new AABB();

  private function insertLeaf(leaf_index : Int) : Void {
    var leaf : DynamicTreeNode = m_nodes[leaf_index];
    if (m_root == null) {
      m_root = leaf;
      m_root.parent = null;
      return;
    }

    // find the best sibling
    var leafAABB : AABB = leaf.aabb;
    var index : DynamicTreeNode = m_root;
    while (index.child1 != null) {
      var node : DynamicTreeNode = index;
      var child1 : DynamicTreeNode = node.child1;
      var child2 : DynamicTreeNode = node.child2;

      var area : Float = node.aabb.getPerimeter();

      combinedAABB.combine(node.aabb, leafAABB);
      var combinedArea : Float = combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      var cost : Float = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      var inheritanceCost : Float = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      var cost1 : Float;
      if (child1.child1 == null) {
        combinedAABB.combine(leafAABB, child1.aabb);
        cost1 = combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        combinedAABB.combine(leafAABB, child1.aabb);
        var oldArea : Float = child1.aabb.getPerimeter();
        var newArea : Float = combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      var cost2 : Float;
      if (child2.child1 == null) {
        combinedAABB.combine(leafAABB, child2.aabb);
        cost2 = combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        combinedAABB.combine(leafAABB, child2.aabb);
        var oldArea : Float = child2.aabb.getPerimeter();
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

    var sibling : DynamicTreeNode = index;
    var oldParent : DynamicTreeNode = m_nodes[sibling.id].parent;
    var newParent : DynamicTreeNode = allocateNode();
    newParent.parent = oldParent;
    newParent.userData = null;
    newParent.aabb.combine(leafAABB, sibling.aabb);
    newParent.height = sibling.height + 1;

    if (oldParent != null) {
      // The sibling was not the root.
      if (oldParent.child1 == sibling) {
        oldParent.child1 = newParent;
      } else {
        oldParent.child2 = newParent;
      }

      newParent.child1 = sibling;
      newParent.child2 = leaf;
      sibling.parent = newParent;
      leaf.parent = newParent;
    } else {
      // The sibling was the root.
      newParent.child1 = sibling;
      newParent.child2 = leaf;
      sibling.parent = newParent;
      leaf.parent = newParent;
      m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = leaf.parent;
    while (index != null) {
      index = balance(index);

      var child1 : DynamicTreeNode = index.child1;
      var child2 : DynamicTreeNode = index.child2;


      index.height = 1 + MathUtils.max(child1.height, child2.height);
      index.aabb.combine(child1.aabb, child2.aabb);

      index = index.parent;
    }
    // validate();
  }

  private function removeLeaf(leaf : DynamicTreeNode) : Void {
    if (leaf == m_root) {
      m_root = null;
      return;
    }

    var parent : DynamicTreeNode = leaf.parent;
    var grandParent : DynamicTreeNode = parent.parent;
    var sibling : DynamicTreeNode;
    if (parent.child1 == leaf) {
      sibling = parent.child2;
    } else {
      sibling = parent.child1;
    }

    if (grandParent != null) {
      // Destroy parent and connect sibling to grandParent.
      if (grandParent.child1 == parent) {
        grandParent.child1 = sibling;
      } else {
        grandParent.child2 = sibling;
      }
      sibling.parent = grandParent;
      freeNode(parent);

      // Adjust ancestor bounds.
      var index : DynamicTreeNode = grandParent;
      while (index != null) {
        index = balance(index);

        var child1 : DynamicTreeNode = index.child1;
        var child2 : DynamicTreeNode = index.child2;

        index.aabb.combine(child1.aabb, child2.aabb);
        index.height = 1 + MathUtils.max(child1.height, child2.height);

        index = index.parent;
      }
    } else {
      m_root = sibling;
      sibling.parent = null;
      freeNode(parent);
    }

    // validate();
  }

  // Perform a left or right rotation if node A is imbalanced.
  // Returns the new root index.
  private function balance(iA : DynamicTreeNode) : DynamicTreeNode {

    var A : DynamicTreeNode = iA;
    if (A.child1 == null || A.height < 2) {
      return iA;
    }

    var iB : DynamicTreeNode = A.child1;
    var iC : DynamicTreeNode = A.child2;

    var B : DynamicTreeNode = iB;
    var C : DynamicTreeNode = iC;

    var balance : Int = C.height - B.height;

    // Rotate C up
    if (balance > 1) {
      var iF : DynamicTreeNode = C.child1;
      var iG : DynamicTreeNode = C.child2;
      var F : DynamicTreeNode = iF;
      var G : DynamicTreeNode = iG;

      // Swap A and C
      C.child1 = iA;
      C.parent = A.parent;
      A.parent = iC;

      // A's old parent should point to C
      if (C.parent != null) {
        if (C.parent.child1 == iA) {
          C.parent.child1 = iC;
        } else {
          C.parent.child2 = iC;
        }
      } else {
        m_root = iC;
      }

      // Rotate
      if (F.height > G.height) {
        C.child2 = iF;
        A.child2 = iG;
        G.parent = iA;
        A.aabb.combine(B.aabb, G.aabb);
        C.aabb.combine(A.aabb, F.aabb);

        A.height = 1 + MathUtils.max(B.height, G.height);
        C.height = 1 + MathUtils.max(A.height, F.height);
      } else {
        C.child2 = iG;
        A.child2 = iF;
        F.parent = iA;
        A.aabb.combine(B.aabb, F.aabb);
        C.aabb.combine(A.aabb, G.aabb);

        A.height = 1 + MathUtils.max(B.height, F.height);
        C.height = 1 + MathUtils.max(A.height, G.height);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      var iD : DynamicTreeNode = B.child1;
      var iE : DynamicTreeNode = B.child2;
      var D : DynamicTreeNode = iD;
      var E : DynamicTreeNode = iE;

      // Swap A and B
      B.child1 = iA;
      B.parent = A.parent;
      A.parent = iB;

      // A's old parent should point to B
      if (B.parent != null) {
        if (B.parent.child1 == iA) {
          B.parent.child1 = iB;
        } else {
          B.parent.child2 = iB;
        }
      } else {
        m_root = iB;
      }

      // Rotate
      if (D.height > E.height) {
        B.child2 = iD;
        A.child1 = iE;
        E.parent = iA;
        A.aabb.combine(C.aabb, E.aabb);
        B.aabb.combine(A.aabb, D.aabb);

        A.height = 1 + MathUtils.max(C.height, E.height);
        B.height = 1 + MathUtils.max(A.height, D.height);
      } else {
        B.child2 = iE;
        A.child1 = iD;
        D.parent = iA;
        A.aabb.combine(C.aabb, D.aabb);
        B.aabb.combine(A.aabb, E.aabb);

        A.height = 1 + MathUtils.max(C.height, D.height);
        B.height = 1 + MathUtils.max(A.height, E.height);
      }

      return iB;
    }

    return iA;
  }

  private function validateStructure(node : DynamicTreeNode) : Void {
    if (node == null) {
      return;
    }

    if (node == m_root) {
    }

    var child1 : DynamicTreeNode = node.child1;
    var child2 : DynamicTreeNode = node.child2;

    if (node.child1 == null) {
      return;
    }



    validateStructure(child1);
    validateStructure(child2);
  }

  private function validateMetrics(node : DynamicTreeNode) : Void {
    if (node == null) {
      return;
    }

    var child1 : DynamicTreeNode = node.child1;
    var child2 : DynamicTreeNode = node.child2;

    if (node.child1 == null) {
      return;
    }


    var height1 : Int = child1.height;
    var height2 : Int = child2.height;
    var height : Int;
    height = 1 + MathUtils.max(height1, height2);

    var aabb : AABB = new AABB();
    aabb.combine(child1.aabb, child2.aabb);


    validateMetrics(child1);
    validateMetrics(child2);
  }

  public function drawTree(argDraw : DebugDraw) : Void {
    if (m_root == null) {
      return;
    }
    var height : Int = computeHeight();
    drawTree2(argDraw, m_root, 0, height);
  }

  private var color : Color3f = new Color3f();
  private var textVec : Vec2 = new Vec2();

  public function drawTree2(argDraw : DebugDraw, node : DynamicTreeNode, spot : Int, height : Int) : Void {
    node.aabb.getVertices(drawVecs);

    color.set(1, (height - spot) * 1 / height, (height - spot) * 1 / height);
    argDraw.drawPolygon(drawVecs, 4, color);

    argDraw.getViewportTranform().getWorldToScreen(node.aabb.upperBound, textVec);
    argDraw.drawString(textVec.x, textVec.y, node.id + "-" + (spot + 1) + "/" + height, color);

    if (node.child1 != null) {
      drawTree2(argDraw, node.child1, spot + 1, height);
    }
    if (node.child2 != null) {
      drawTree2(argDraw, node.child2, spot + 1, height);
    }
  }
}

