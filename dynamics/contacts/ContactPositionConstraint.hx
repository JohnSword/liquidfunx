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
package box2d.dynamics.contacts;

import box2d.collision.Manifold.ManifoldType;
import box2d.common.Settings;
import box2d.common.Vec2;

 class ContactPositionConstraint {
  public var localPoints : Array<Vec2> = new Array<Vec2>();
  public var localNormal : Vec2 = new Vec2();
  public var localPoint : Vec2 = new Vec2();
  public var indexA : Int;
  public var indexB : Int;
  public var invMassA : Float;
  public var invMassB : Float;
  public var localCenterA : Vec2 = new Vec2();
  public var localCenterB : Vec2 = new Vec2();
  public var invIA : Float;
  public var invIB : Float;
  public var type : ManifoldType;
  public var radiusA : Float;
  public var radiusB : Float;
  public var pointCount : Int;

  public function new() {
    for(i in 0 ... Settings.maxManifoldPoints) {
      localPoints[i] = new Vec2();
    }
  }
}

