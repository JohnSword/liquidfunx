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
package box2d.pooling.normal;

import box2d.pooling.IOrderedStack;
import haxe.ds.Vector;

class CircleStack<E> implements IOrderedStack<E> {

  private var pool : Vector<Dynamic>;
  private var index : Int = 0;
  private var size : Int = 0;
  private var container : Vector<Dynamic>;

  public function new(argStackSize : Int, argContainerSize : Int) {
    size = argStackSize;
    pool = new Vector<Dynamic>(argStackSize);
    for(i in 0 ... argStackSize) {
      pool[i] = newInstance();
    }
    index = 0;
    container = new Vector<Dynamic>(argContainerSize);
  }

  public function pop() : E {
    index++;
    if(index >= size){
      index = 0;
    }
    return cast pool[index];
  }

  public function popArray(argNum : Int) : Array<E> {
    if(index + argNum < size){
      // TODO: array copy
      // System.arraycopy(pool, index, container, 0, argNum);
      index += argNum;
    }else{
      var overlap : Int = (index + argNum) - size;
      // TODO: array copys
      // System.arraycopy(pool, index, container, 0, argNum - overlap);
      // System.arraycopy(pool, 0, container, argNum - overlap, overlap);
      index = overlap;
    }
    return cast container;
  }

  public function push(argNum : Int) : Void {}

  /** Creates a new instance of the object contained by this stack. */
  public function newInstance() : E {
    return null;
  }
}

