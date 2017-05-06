package box2d.collision;

import box2d.common.Vec2;

/**
 * GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
 */
class SimplexVertex {
    public var wA : Vec2 = new Vec2();
    public var wB : Vec2 = new Vec2();
    public var w : Vec2 = new Vec2();
    public var a : Float = 0; // barycentric coordinate for closest point
    public var indexA : Int = 0; // wA index
    public var indexB : Int = 0; // wB index

    public function new() {}

    public function set(sv : SimplexVertex) : Void {
        wA.setVec(sv.wA);
        wB.setVec(sv.wB);
        w.setVec(sv.w);
        a = sv.a;
        indexA = sv.indexA;
        indexB = sv.indexB;
    }
}