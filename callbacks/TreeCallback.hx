package box2d.callbacks;

/**
 * callback for {@link DynamicTree}
 * @author Daniel Murphy
 *
 */
interface TreeCallback {
	
	/**
	 * Callback from a query request.  
	 * @param proxyId the id of the proxy
	 * @return if the query should be continued
	 */
	function treeCallback(proxyId : Int) : Bool;
}