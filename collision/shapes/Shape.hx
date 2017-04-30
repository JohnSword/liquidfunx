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
import box2d.common.Transform;
import box2d.common.Vec2;

/**
 * A shape is used for collision detection. You can create a shape however you like. Shapes used for
 * simulation in World are created automatically when a Fixture is created. Shapes may encapsulate a
 * one or more child shapes.
 */
class Shape {

  public var m_type : ShapeType;
  public var m_radius : Float;

  public function new(type : ShapeType) {
    this.m_type = type;
  }

  /**
   * Get the type of this shape. You can use this to down cast to the concrete shape.
   * 
   * @return the shape type.
   */
  public function getType() : ShapeType {
    return m_type;
  }

  /**
   * The radius of the underlying shape. This can refer to different things depending on the shape
   * implementation
   * 
   * @return
   */
  public function getRadius() : Float {
    return m_radius;
  }

  /**
   * Sets the radius of the underlying shape. This can refer to different things depending on the
   * implementation
   * 
   * @param radius
   */
  public function setRadius(radius : Float) : Void {
    this.m_radius = radius;
  }

  /**
   * Get the number of child primitives
   * 
   * @return
   */
  public function getChildCount() : Int {
    return null;
  }

  /**
   * Test a point for containment in this shape. This only works for convex shapes.
   * 
   * @param xf the shape world transform.
   * @param p a point in world coordinates.
   */
  public function testPoint(xf : Transform, p : Vec2) : Bool {
    return false;
  }

  /**
   * Cast a ray against a child shape.
   * 
   * @param argOutput the ray-cast results.
   * @param argInput the ray-cast input parameters.
   * @param argTransform the transform to be applied to the shape.
   * @param argChildIndex the child shape index
   * @return if hit
   */
  public function raycast(output : RayCastOutput, input : RayCastInput, transform : Transform, childIndex : Int) : Bool {
    return false;
  }


  /**
   * Given a transform, compute the associated axis aligned bounding box for a child shape.
   * 
   * @param argAabb returns the axis aligned box.
   * @param argXf the world transform of the shape.
   */
  public function computeAABB(aabb : AABB, xf : Transform, childIndex : Int) : Void {
    
  }

  /**
   * Compute the mass properties of this shape using its dimensions and density. The inertia tensor
   * is computed about the local origin.
   * 
   * @param massData returns the mass data for this shape.
   * @param density the density in kilograms per meter squared.
   */
  public function computeMass(massData : MassData, density : Float) : Void {
    
  }

  /**
   * Compute the distance from the current shape to the specified point. This only works for convex
   * shapes.
   * 
   * @param xf the shape world transform.
   * @param p a point in world coordinates.
   * @param normalOut returns the direction in which the distance increases.
   * @return distance returns the distance from the current shape.
   */
  public function computeDistanceToOut(xf : Transform, p : Vec2, childIndex : Int, normalOut : Vec2) : Float {
    return null;
  }

  public function clone() : Shape {
    return null;
  }
}

