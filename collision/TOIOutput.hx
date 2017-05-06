package box2d.collision;

import box2d.collision.TimeOfImpact;

/**
 * Output parameters for TimeOfImpact
 * 
 * @author daniel
 */
class TOIOutput {
    public var state : TOIOutputState;
    public var t : Float = 0;
    public function new() {}
}