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

 class Profile {
  public static var LONG_AVG_NUMS : Int = 20;
  public static var LONG_FRACTION : Float = 1 / LONG_AVG_NUMS;
  public static var SHORT_AVG_NUMS : Int = 5;
  public static var SHORT_FRACTION : Float = 1 / SHORT_AVG_NUMS;

  public var step : ProfileEntry = new ProfileEntry();
  public var stepInit : ProfileEntry = new ProfileEntry();
  public var collide : ProfileEntry = new ProfileEntry();
  public var solveParticleSystem : ProfileEntry = new ProfileEntry();
  public var solve : ProfileEntry = new ProfileEntry();
  public var solveInit : ProfileEntry = new ProfileEntry();
  public var solveVelocity : ProfileEntry = new ProfileEntry();
  public var solvePosition : ProfileEntry = new ProfileEntry();
  public var broadphase : ProfileEntry = new ProfileEntry();
  public var solveTOI : ProfileEntry = new ProfileEntry();

  public function new() {}

  public function toDebugStrings(strings : List<String>) : Void {
    strings.add("Profile:");
    strings.add(" step: " + step);
    strings.add("  init: " + stepInit);
    strings.add("  collide: " + collide);
    strings.add("  particles: " + solveParticleSystem);
    strings.add("  solve: " + solve);
    strings.add("   solveInit: " + solveInit);
    strings.add("   solveVelocity: " + solveVelocity);
    strings.add("   solvePosition: " + solvePosition);
    strings.add("   broadphase: " + broadphase);
    strings.add("  solveTOI: " + solveTOI);
  }
}

