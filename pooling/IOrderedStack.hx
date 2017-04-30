package box2d.pooling;

interface IOrderedStack<E> {

	/**
	 * Returns the next object in the pool
	 * @return
	 */
	function pop() : E;

	/**
	 * Returns the next 'argNum' objects in the pool
	 * in an array
	 * @param argNum
	 * @return an array containing the next pool objects in
	 * 		   items 0-argNum.  Array length and uniqueness not
	 * 		   guaranteed.
	 */
	function popArray(argNum : Int) : Array<E>;

	/**
	 * Tells the stack to take back the last 'argNum' items
	 * @param argNum
	 */
	function push(argNum : Int) : Void;

}