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

import box2d.pooling.IDynamicStack;
import haxe.ds.Vector;

class MutableStack<E> implements IDynamicStack<E> {

  private var stack : Vector<E>;
  private var index : Int = 0;
  private var size : Int = 0;

  public function new(argInitSize : Int) {
    index = 0;
    stack = null;
    index = 0;
    extendStack(argInitSize);
  }

  private function extendStack(argSize : Int) : Void {
    var newStack : Vector<E> = newArray(argSize);
    if (stack != null) {
      // TODO: array copy
      Vector.blit(stack, 0, newStack, 0, size);
      // System.arraycopy(stack, 0, newStack, 0, size);
    }
    for(i in 0 ... newStack.length) {
      newStack[i] = newInstance();
    }
    stack = newStack;
    size = newStack.length;
  }

  public function pop() : E {
    if (index >= size) {
      extendStack(size * 2);
    }
    var idx : Int = index++;
    if(stack[idx] == null) {
      stack[idx] = newInstance();
    }
    return stack[idx];
    // return stack[index++];
  }

  public function push(argObject : E) : Void {
    stack[--index] = argObject;
  }

  /** Creates a new instance of the object contained by this stack. */
  dynamic public function newInstance() : E {
    return null;
  }
  
  dynamic public function newArray<E>(size : Int) : Dynamic {
    return cast new Vector<Dynamic>(size);
  }
}

