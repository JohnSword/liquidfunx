package box2d.particle;

/** Connection between two particles */
class Pair {
    public var indexA : Int = 0;
    public var indexB : Int = 0;
    public var flags : Int = 0;
    public var strength : Float = 0;
    public var distance : Float = 0;
    public function new() {}
}