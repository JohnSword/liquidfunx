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
/**
 * Created at 3:26:14 AM Jan 11, 2011
 */
package box2d.pooling.normal;

import box2d.collision.AABB;
import box2d.collision.Collision;
import box2d.collision.Distance;
import box2d.collision.TimeOfImpact;
import box2d.common.Mat22;
import box2d.common.Mat33;
import box2d.common.Rot;
import box2d.common.Settings;
import box2d.common.Vec2;
import box2d.common.Vec3;
import box2d.dynamics.contacts.ChainAndCircleContact;
import box2d.dynamics.contacts.ChainAndPolygonContact;
import box2d.dynamics.contacts.CircleContact;
import box2d.dynamics.contacts.Contact;
import box2d.dynamics.contacts.EdgeAndCircleContact;
import box2d.dynamics.contacts.EdgeAndPolygonContact;
import box2d.dynamics.contacts.PolygonAndCircleContact;
import box2d.dynamics.contacts.PolygonContact;
import box2d.pooling.IDynamicStack;
import box2d.pooling.IWorldPool;

import haxe.ds.Vector;

/**
 * Provides object pooling for all objects used in the engine. Objects retrieved from here should
 * only be used temporarily, and then pushed back (with the exception of arrays).
 * 
 * @author Daniel Murphy
 */
 class DefaultWorldPool implements IWorldPool {

  private var vecs : OrderedStack<Vec2>;
  private var vec3s : OrderedStack<Vec3>;
  private var mats : OrderedStack<Mat22>;
  private var mat33s : OrderedStack<Mat33>;
  private var aabbs : OrderedStack<AABB>;
  private var rots : OrderedStack<Rot>;

  private var afloats : Map<Int, Vector<Float>> = new Map<Int, Vector<Float>>();
  private var aints : Map<Int, Vector<Int>> = new Map<Int, Vector<Int>>();
  private var avecs : Map<Int, Vector<Vec2>> = new Map<Int, Vector<Vec2>>();

  private var world : IWorldPool = this;

  private var pcstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var ccstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var cpstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var ecstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var epstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var chcstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);
  private var chpstack : MutableStack<Contact> = new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE);

  private var collision : Collision;
  private var toi : TimeOfImpact;
  private var dist : Distance;

  public function new(argSize : Int, argContainerSize : Int) {

    pcstack.newInstance = function() : Contact {
      return new PolygonContact(world);
    }
    pcstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<PolygonContact>(size);
    }

    ccstack.newInstance = function() : Contact {
      return new CircleContact(world);
    }
    ccstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<CircleContact>(size);
    }

    cpstack.newInstance = function() : Contact {
      return new PolygonAndCircleContact(world);
    }
    cpstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<PolygonAndCircleContact>(size);
    }

    ecstack.newInstance = function() : Contact {
      return cast new EdgeAndCircleContact(world);
    }
    ecstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<EdgeAndCircleContact>(size);
    }

    epstack.newInstance = function() : Contact {
      return new EdgeAndPolygonContact(world);
    }
    epstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<EdgeAndPolygonContact>(size);
    }

    chcstack.newInstance = function() : Contact {
      return new ChainAndCircleContact(world);
    }
    chcstack.newArray = function(size : Int) : Vector<Contact> {
      return cast new Vector<ChainAndCircleContact>(size);
    }

    chpstack.newInstance = function() : Contact {
      return new ChainAndPolygonContact(world);
    }
    chpstack.newArray = function(size : Int) : Vector<Contact> { 
        return cast new Vector<ChainAndPolygonContact>(size);
    }

    vecs = new OrderedStack<Vec2>(argSize, argContainerSize);
    vecs.newInstance = function() : Vec2 {
      return new Vec2();
    }
    
    vec3s = new OrderedStack<Vec3>(argSize, argContainerSize);
    vec3s.newInstance = function() : Vec3 {
      return new Vec3();
    }

    mats = new OrderedStack<Mat22>(argSize, argContainerSize);
    mats.newInstance = function() : Mat22 {
      return new Mat22();
    }
    
    aabbs = new OrderedStack<AABB>(argSize, argContainerSize);
    aabbs.newInstance = function() : AABB {
      return new AABB();
    }

    rots = new OrderedStack<Rot>(argSize, argContainerSize);
    rots.newInstance = function() : Rot {
      return new Rot();
    }

    mat33s = new OrderedStack<Mat33>(argSize, argContainerSize);
    mat33s.newInstance = function() : Mat33 {
      return new Mat33();
    }

    dist = new Distance();
    collision = new Collision(this);
    toi = new TimeOfImpact(this);
  }

  public function getPolyContactStack() : IDynamicStack<Contact> {
    return pcstack;
  }

  public function getCircleContactStack() : IDynamicStack<Contact> {
    return ccstack;
  }

  public function getPolyCircleContactStack() : IDynamicStack<Contact> {
    return cpstack;
  }

  public function getEdgeCircleContactStack() : IDynamicStack<Contact> {
    return ecstack;
  }

  public function getEdgePolyContactStack() : IDynamicStack<Contact> {
    return epstack;
  }

  public function getChainCircleContactStack() : IDynamicStack<Contact> {
    return chcstack;
  }

  public function getChainPolyContactStack() : IDynamicStack<Contact> {
    return chpstack;
  }

  public function popVec2() : Vec2 {
    return vecs.pop();
  }

  public function popVec2Array(argNum : Int) : Vector<Vec2> {
    return vecs.popArray(argNum);
  }

  public function pushVec2(argNum : Int) : Void {
    vecs.push(argNum);
  }

  public function popVec3() : Vec3 {
    return vec3s.pop();
  }

  public function popVec3Array(argNum : Int) : Vector<Vec3> {
    return vec3s.popArray(argNum);
  }

  public function pushVec3(argNum : Int) : Void {
    vec3s.push(argNum);
  }

  public function popMat22() : Mat22 {
    return mats.pop();
  }

  public function popMat22Array(argNum : Int) : Vector<Mat22> {
    return mats.popArray(argNum);
  }

  public function pushMat22(argNum : Int) : Void {
    mats.push(argNum);
  }

  public function popMat33() : Mat33 {
    return mat33s.pop();
  }

  public function pushMat33(argNum : Int) : Void {
    mat33s.push(argNum);
  }

  public function popAABB() : AABB {
    return aabbs.pop();
  }

  public function popAABBArray(argNum : Int) : Vector<AABB> {
    return aabbs.popArray(argNum);
  }

  public function pushAABB(argNum : Int) : Void {
    aabbs.push(argNum);
  }

  public function popRot() : Rot {
    return rots.pop();
  }

  public function pushRot(num : Int) : Void {
    rots.push(num);
  }

  public function getCollision() : Collision {
    return collision;
  }

  public function getTimeOfImpact() : TimeOfImpact {
    return toi;
  }

  public function getDistance() : Distance {
    return dist;
  }

  public function getFloatArray(argLength : Int) : Vector<Float> {
    if (!afloats.exists(argLength)) {
      afloats.set(argLength, new Vector<Float>(argLength));
    }
    return afloats.get(argLength);
  }

  public function getIntArray(argLength : Int) : Vector<Int> {
    if (!aints.exists(argLength)) {
      aints.set(argLength, new Vector<Int>(argLength));
    }
    return aints.get(argLength);
  }

  public function getVec2Array(argLength : Int) : Vector<Vec2> {
    if (!avecs.exists(argLength)) {
      var ray : Vector<Vec2> = new Vector(argLength);
      for(i in 0 ... argLength) {
        ray[i] = new Vec2();
      }
      avecs.set(argLength, ray);
    }
    return avecs.get(argLength);
  }

  public function popAABBs(num : Int) : Vector<AABB> {
    return null;
  }

  public function popMat22s(num : Int) : Vector<Mat22> {
    return null;
  }

  public function popVec2s(num : Int) : Vector<Vec2> {
    return null;
  }

  public function popVec3s(num : Int) : Vector<Vec3> {
    return null;
  }

}

