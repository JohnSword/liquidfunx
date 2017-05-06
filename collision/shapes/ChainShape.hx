/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
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
package box2d.collision.shapes;


import box2d.collision.AABB;
import box2d.collision.RayCastInput;
import box2d.collision.RayCastOutput;
import box2d.common.MathUtils;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Transform;
import box2d.common.Vec2;
import haxe.ds.Vector;

/**
 * A chain shape is a free form sequence of line segments. The chain has two-sided collision, so you
 * can use inside and outside collision. Therefore, you may use any winding order. Connectivity
 * information is used to create smooth collisions. WARNING: The chain will not collide properly if
 * there are self-intersections.
 * 
 * @author Daniel
 */
 class ChainShape extends Shape {

  public var m_vertices : Vector<Vec2>;
  public var m_count : Int = 0;
  public var m_nextVertex : Vec2 = new Vec2();
  public var m_prevVertex : Vec2 = new Vec2();
  public var m_hasNextVertex : Bool = false;
  public var m_hasPrevVertex : Bool = false;

  private var pool0 : EdgeShape = new EdgeShape();

  public function new() {
    super(ShapeType.CHAIN);
    m_vertices = null;
    m_radius = Settings.polygonRadius;
    m_count = 0;
  }

  public function clear() : Void {
    m_vertices = null;
    m_count = 0;
  }

  override public function getChildCount() : Int {
    return m_count - 1;
  }

  /**
   * Get a child edge.
   */
  public function getChildEdge(edge : EdgeShape, index : Int) : Void {
    edge.m_radius = m_radius;

    var v0 : Vec2 = m_vertices[index + 0];
    var v1 : Vec2 = m_vertices[index + 1];
    edge.m_vertex1.x = v0.x;
    edge.m_vertex1.y = v0.y;
    edge.m_vertex2.x = v1.x;
    edge.m_vertex2.y = v1.y;

    if (index > 0) {
      var v : Vec2 = m_vertices[index - 1];
      edge.m_vertex0.x = v.x;
      edge.m_vertex0.y = v.y;
      edge.m_hasVertex0 = true;
    } else {
      edge.m_vertex0.x = m_prevVertex.x;
      edge.m_vertex0.y = m_prevVertex.y;
      edge.m_hasVertex0 = m_hasPrevVertex;
    }

    if (index < m_count - 2) {
      var v : Vec2 = m_vertices[index + 2];
      edge.m_vertex3.x = v.x;
      edge.m_vertex3.y = v.y;
      edge.m_hasVertex3 = true;
    } else {
      edge.m_vertex3.x = m_nextVertex.x;
      edge.m_vertex3.y = m_nextVertex.y;
      edge.m_hasVertex3 = m_hasNextVertex;
    }
  }

  override public function computeDistanceToOut(xf : Transform, p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    var edge : EdgeShape = pool0;
    getChildEdge(edge, childIndex);
    return edge.computeDistanceToOut(xf, p, 0, normalOut);
  }

  override public function testPoint(xf : Transform, p : Vec2) : Bool {
    return false;
  }

  override public function raycast(output : RayCastOutput, input : RayCastInput, xf : Transform, childIndex : Int) : Bool {

    var edgeShape : EdgeShape = pool0;

    var i1 : Int = childIndex;
    var i2 : Int = childIndex + 1;
    if (i2 == m_count) {
      i2 = 0;
    }
    var v : Vec2 = m_vertices[i1];
    edgeShape.m_vertex1.x = v.x;
    edgeShape.m_vertex1.y = v.y;
    var v1 : Vec2 = m_vertices[i2];
    edgeShape.m_vertex2.x = v1.x;
    edgeShape.m_vertex2.y = v1.y;

    return edgeShape.raycast(output, input, xf, 0);
  }

  override public function computeAABB(aabb : AABB, xf : Transform, childIndex : Int) : Void {
    var lower : Vec2 = aabb.lowerBound;
    var upper : Vec2 = aabb.upperBound;

    var i1 : Int = childIndex;
    var i2 : Int = childIndex + 1;
    if (i2 == m_count) {
      i2 = 0;
    }

    var vi1 : Vec2 = m_vertices[i1];
    var vi2 : Vec2 = m_vertices[i2];
    var xfq : Rot = xf.q;
    var xfp : Vec2 = xf.p;
    var v1x : Float = (xfq.c * vi1.x - xfq.s * vi1.y) + xfp.x;
    var v1y : Float = (xfq.s * vi1.x + xfq.c * vi1.y) + xfp.y;
    var v2x : Float = (xfq.c * vi2.x - xfq.s * vi2.y) + xfp.x;
    var v2y : Float = (xfq.s * vi2.x + xfq.c * vi2.y) + xfp.y;

    lower.x = v1x < v2x ? v1x : v2x;
    lower.y = v1y < v2y ? v1y : v2y;
    upper.x = v1x > v2x ? v1x : v2x;
    upper.y = v1y > v2y ? v1y : v2y;
  }

  override public function computeMass(massData : MassData, density : Float) : Void {
    massData.mass = 0.0;
    massData.center.setZero();
    massData.I = 0.0;
  }

  override public function clone() : Shape {
    var clone : ChainShape = new ChainShape();
    clone.createChain(m_vertices, m_count);
    clone.m_prevVertex.setVec(m_prevVertex);
    clone.m_nextVertex.setVec(m_nextVertex);
    clone.m_hasPrevVertex = m_hasPrevVertex;
    clone.m_hasNextVertex = m_hasNextVertex;
    return clone;
  }

  /**
   * Create a loop. This automatically adjusts connectivity.
   * 
   * @param vertices an array of vertices, these are copied
   * @param count the vertex count
   */
  public function createLoop(vertices : Vector<Vec2>, count : Int) : Void {
    m_count = count + 1;
    m_vertices = new Vector<Vec2>(m_count);
    for(i in 1 ... count) {
      var v1 : Vec2 = vertices[i - 1];
      var v2 : Vec2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) < Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for(i in 0 ... count) {
      m_vertices[i] = new Vec2().setVec(vertices[i]);
    }
    m_vertices[count] = new Vec2().setVec(m_vertices[0]);
    m_prevVertex.setVec(m_vertices[m_count - 2]);
    m_nextVertex.setVec(m_vertices[1]);
    m_hasPrevVertex = true;
    m_hasNextVertex = true;
  }

  /**
   * Create a chain with isolated end vertices.
   * 
   * @param vertices an array of vertices, these are copied
   * @param count the vertex count
   */
  public function createChain(vertices : Vector<Vec2>, count : Int) : Void {
    m_count = count;
    m_vertices = new Vector<Vec2>(m_count);
    for(i in 1 ... m_count) {
      var v1 : Vec2 = vertices[i - 1];
      var v2 : Vec2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) < Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for(i in 0 ... m_count) {
      m_vertices[i] = new Vec2().setVec(vertices[i]);
    }
    m_hasPrevVertex = false;
    m_hasNextVertex = false;

    m_prevVertex.setZero();
    m_nextVertex.setZero();
  }

  /**
   * Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
   * 
   * @param prevVertex
   */
  public function setPrevVertex(prevVertex : Vec2) : Void {
    m_prevVertex.setVec(prevVertex);
    m_hasPrevVertex = true;
  }

  /**
   * Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
   * 
   * @param nextVertex
   */
  public function setNextVertex(nextVertex : Vec2) : Void {
    m_nextVertex.setVec(nextVertex);
    m_hasNextVertex = true;
  }
}

