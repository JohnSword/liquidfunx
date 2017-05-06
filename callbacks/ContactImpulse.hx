package box2d.callbacks;

/**
 * Contact impulses for reporting. Impulses are used instead of forces because sub-step forces may
 * approach infinity for rigid body collisions. These match up one-to-one with the contact points in
 * b2Manifold.
 * 
 * @author Daniel Murphy
 */
class ContactImpulse {

    public var normalImpulses : Array<Float> = new Array<Float>();
    public var tangentImpulses : Array<Float> = new Array<Float>();
    public var count : Int = 0;

    public function new() {

    }

}