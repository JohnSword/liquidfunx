package box2d.collision.broadphase;

import box2d.common.Vec2;
import box2d.callbacks.TreeCallback;
import box2d.callbacks.TreeRayCastCallback;
import box2d.callbacks.PairCallback;
import box2d.callbacks.DebugDraw;

interface BroadPhase {

    /**
     * Create a proxy with an initial AABB. Pairs are not reported until updatePairs is called.
     * 
     * @param aabb
     * @param userData
     * @return
     */
    function createProxy(aabb : AABB, userData : Dynamic) : Int;

    /**
     * Destroy a proxy. It is up to the client to remove any pairs.
     * 
     * @param proxyId
     */
    function destroyProxy(proxyId : Int) : Void;

    /**
     * Call MoveProxy as many times as you like, then when you are done call UpdatePairs to finalized
     * the proxy pairs (for your time step).
     */
    function moveProxy(proxyId : Int, aabb : AABB, displacement : Vec2) : Void;

    function touchProxy(proxyId : Int) : Void;

    function getUserData(proxyId : Int) : Dynamic;

    function getFatAABB(proxyId : Int) : AABB;

    function testOverlap(proxyIdA : Int, proxyIdB : Int) : Bool;

    /**
     * Get the number of proxies.
     * 
     * @return
     */
    function getProxyCount() : Int;

    function drawTree(argDraw : DebugDraw) : Void;

    /**
     * Update the pairs. This results in pair callbacks. This can only add pairs.
     * 
     * @param callback
     */
     function updatePairs(callback : PairCallback) : Void;

    /**
     * Query an AABB for overlapping proxies. The callback class is called for each proxy that
     * overlaps the supplied AABB.
     * 
     * @param callback
     * @param aabb
     */
     function query(callback : TreeCallback, aabb : AABB) : Void;

    /**
     * Ray-cast against the proxies in the tree. This relies on the callback to perform a exact
     * ray-cast in the case were the proxy contains a shape. The callback also performs the any
     * collision filtering. This has performance roughly equal to k * log(n), where k is the number of
     * collisions and n is the number of proxies in the tree.
     * 
     * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
     * @param callback a callback class that is called for each proxy that is hit by the ray.
     */
     function raycast(callback : TreeRayCastCallback, input : RayCastInput) : Void;

    /**
     * Get the height of the embedded tree.
     * 
     * @return
     */
    function getTreeHeight() : Int;

    function getTreeBalance() : Int;

    function getTreeQuality() : Float;
}