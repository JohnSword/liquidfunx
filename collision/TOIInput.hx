package box2d.collision;

import box2d.common.Sweep;
import box2d.collision.DistanceProxy;

/**
 * Input parameters for TOI
 * 
 * @author Daniel Murphy
 */
class TOIInput {
    public var proxyA : DistanceProxy = new DistanceProxy();
    public var proxyB : DistanceProxy = new DistanceProxy();
    public var sweepA : Sweep = new Sweep();
    public var sweepB : Sweep = new Sweep();
    /**
     * defines sweep interval [0, tMax]
     */
    public var tMax : Float;
    public function new () {}
}