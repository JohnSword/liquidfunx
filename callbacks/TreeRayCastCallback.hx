package box2d.callbacks;

import box2d.collision.RayCastInput;

/**
 * callback for {@link DynamicTree}
 * @author Daniel Murphy
 *
 */
interface TreeRayCastCallback {
	/**
	 * 
	 * @param input
	 * @param nodeId
	 * @return the fraction to the node
	 */
	function raycastCallback(input : RayCastInput, nodeId : Int) : Float;
}