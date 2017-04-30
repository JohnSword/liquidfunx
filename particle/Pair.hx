package box2d.particle;

/** Connection between two particles */
class Pair {
    public var indexA : Int;
    public var indexB : Int;
    public var flags : Int;
    public var strength : Float;
    public var distance : Float;
    public function new() {}
}