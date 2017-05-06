package box2d.particle;

import box2d.common.Vec2;

/** Connection between three particles */
class Triad {
    public var indexA : Int = 0; 
    public var indexB : Int = 0;
    public var indexC : Int = 0;
    public var flags : Int = 0;
    public var strength : Float = 0;
    public var pa : Vec2 = new Vec2();
    public var pb = new Vec2();
    public var pc = new Vec2();
    public var ka : Float = 0;
    public var kb : Float = 0; 
    public var kc : Float = 0; 
    public var s : Float = 0;
    public function new() {}
}