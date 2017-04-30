package box2d.pooling;

interface IDynamicStack<E> {

	/**
	 * Pops an item off the stack
	 * @return
	 */
    function pop() : E;

	/**
	 * Pushes an item back on the stack
	 * @param argObject
	 */
	function push(argObject : E) : Void;

}