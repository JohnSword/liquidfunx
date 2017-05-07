package box2d.pooling;

interface IDynamicStack {

	/**
	 * Pops an item off the stack
	 * @return
	 */
    function pop() : Dynamic;

	/**
	 * Pushes an item back on the stack
	 * @param argObject
	 */
	function push(argObject : Dynamic) : Void;

}