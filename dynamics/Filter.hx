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

// updated to rev 100
/**
 * This holds contact filtering data.
 * 
 * @author daniel
 */
 class Filter {
	/**
	 * The collision category bits. Normally you would just set one bit.
	 */
	public var categoryBits : Int = 0;
	
	/**
	 * The collision mask bits. This states the categories that this
	 * shape would accept for collision.
	 */
	public var maskBits : Int = 0;
	
	/**
	 * Collision groups allow a certain group of objects to never collide (negative)
	 * or always collide (positive). Zero means no collision group. Non-zero group
	 * filtering always wins against the mask bits.
	 */
	public var groupIndex : Int = 0;
	
	public function new() {
	  categoryBits = 0x0001;
      maskBits = 0xFFFF;
      groupIndex = 0;
    }
	
	public function set(argOther : Filter) : Void {
		categoryBits = argOther.categoryBits;
		maskBits = argOther.maskBits;
		groupIndex = argOther.groupIndex;
	}
}

