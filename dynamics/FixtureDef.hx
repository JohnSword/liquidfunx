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
package box2d.dynamics;

import box2d.collision.shapes.Shape;

/**
 * A fixture definition is used to create a fixture. This class defines an abstract fixture
 * definition. You can reuse fixture definitions safely.
 * 
 * @author daniel
 */
 class FixtureDef {
  /**
   * The shape, this must be set. The shape will be cloned, so you can create the shape on the
   * stack.
   */
  public var shape : Shape = null;

  /**
   * Use this to store application specific fixture data.
   */
  public var userData : Dynamic;

  /**
   * The friction coefficient, usually in the range [0,1].
   */
  public var friction : Float;

  /**
   * The restitution (elasticity) usually in the range [0,1].
   */
  public var restitution : Float;

  /**
   * The density, usually in kg/m^2
   */
  public var density : Float;

  /**
   * A sensor shape collects contact information but never generates a collision response.
   */
  public var isSensor : Bool;

  /**
   * Contact filtering data;
   */
  public var filter : Filter;

  public function new() {
    shape = null;
    userData = null;
    friction = 0.2;
    restitution = 0;
    density = 0;
    filter = new Filter();
    isSensor = false;
  }

  /**
   * The shape, this must be set. The shape will be cloned, so you can create the shape on the
   * stack.
   */
  public function getShape() : Shape {
    return shape;
  }

  /**
   * The shape, this must be set. The shape will be cloned, so you can create the shape on the
   * stack.
   */
  public function setShape(shape : Shape) : Void {
    this.shape = shape;
  }

  /**
   * Use this to store application specific fixture data.
   */
  public function getUserData() : Dynamic {
    return userData;
  }

  /**
   * Use this to store application specific fixture data.
   */
  public function setUserData(userData : Dynamic) : Void {
    this.userData = userData;
  }

  /**
   * The friction coefficient, usually in the range [0,1].
   */
  public function getFriction() : Float {
    return friction;
  }

  /**
   * The friction coefficient, usually in the range [0,1].
   */
  public function setFriction(friction : Float) : Void {
    this.friction = friction;
  }

  /**
   * The restitution (elasticity) usually in the range [0,1].
   */
  public function getRestitution() : Float {
    return restitution;
  }

  /**
   * The restitution (elasticity) usually in the range [0,1].
   */
  public function setRestitution(restitution : Float) : Void {
    this.restitution = restitution;
  }

  /**
   * The density, usually in kg/m^2
   */
  public function getDensity() : Float {
    return density;
  }

  /**
   * The density, usually in kg/m^2
   */
  public function setDensity(density : Float) : Void {
    this.density = density;
  }

  /**
   * A sensor shape collects contact information but never generates a collision response.
   */
  public function getIsSensor() : Bool {
    return isSensor;
  }

  /**
   * A sensor shape collects contact information but never generates a collision response.
   */
  public function setSensor(isSensor : Bool) : Void {
    this.isSensor = isSensor;
  }

  /**
   * Contact filtering data;
   */
  public function getFilter() : Filter {
    return filter;
  }

  /**
   * Contact filtering data;
   */
  public function setFilter(filter : Filter) : Void {
    this.filter = filter;
  }
}

