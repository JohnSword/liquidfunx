package box2d.collision;

import box2d.common.Settings;

/**
 * Used to warm start Distance. Set count to zero on first call.
 * 
 * @author daniel
 */
class SimplexCache {
    /** length or area */
    public var metric : Float;
    public var count : Int;
    /** vertices on shape A */
    public var indexA : Array<Int> = new Array<Int>();
    /** vertices on shape B */
    public var indexB : Array<Int> = new Array<Int>();

    public function new() {
        metric = 0;
        count = 0;
        indexA[0] = Settings.MAX_VALUE_INT;
        indexA[1] = Settings.MAX_VALUE_INT;
        indexA[2] = Settings.MAX_VALUE_INT;
        indexB[0] = Settings.MAX_VALUE_INT;
        indexB[1] = Settings.MAX_VALUE_INT;
        indexB[2] = Settings.MAX_VALUE_INT;
    }

    public function set(sc : SimplexCache) : Void {
        // TODO: implement array copy
        indexA = sc.indexA.copy();
        indexB = sc.indexB.copy();
        // System.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
        // System.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
        metric = sc.metric;
        count = sc.count;
    }
}