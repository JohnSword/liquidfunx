package box2d.particle;

import box2d.common.Vec2;

/** Connection between three particles */
class Triad {
    public var indexA : Int; 
    public var indexB : Int;
    public var indexC : Int;
    public var flags : Int;
    public var strength : Float;
    public var pa : Vec2 = new Vec2();
    public var pb = new Vec2();
    public var pc = new Vec2();
    public var ka : Float;
    public var kb : Float; 
    public var kc : Float; 
    public var s : Float;
    public function new() {}
}