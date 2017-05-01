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
 * Created at 12:52:04 AM Jan 20, 2011
 */
package box2d.pooling.normal;

import haxe.ds.Vector;

/**
 * @author Daniel Murphy
 */
class OrderedStack<E> {

  private var pool : Vector<Dynamic>;
  private var index : Int;
  private var size : Int;
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
    var idx :Int = index++;
    var o : E = pool[idx];
    if(o == null) {
      pool[idx] = newInstance();
    }
    return pool[idx];
  }

  public function popArray(argNum : Int) : Vector<E> {
    // TODO: array copy
    Vector.blit(pool, index, container, 0, argNum);
    // System.arraycopy(pool, index, container, 0, argNum);
    index += argNum;
    return cast container;
  }

  public function push(argNum : Int) : Void {
    index -= argNum;
  }

  /** Creates a new instance of the object contained by this stack. */
  dynamic public function newInstance() : E {
    return null;
  }
}

