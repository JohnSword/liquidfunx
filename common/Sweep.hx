package box2d.common;

class Sweep {

    private static var serialVersionUID : Int = 1;

    /** Local center of mass position */
    public var localCenter : Vec2;
    /** Center world positions */
    public var c0 : Vec2;
    public var c : Vec2;
    /** World angles */
    public var a0 : Float = 0;
    public var a : Float = 0;

    /** Fraction of the current time step in the range [0,1] c0 and a0 are the positions at alpha0. */
    public var alpha0 : Float = 0;

    public function new() {
        localCenter = new Vec2();
        c0 = new Vec2();
        c = new Vec2();
    }

    public function toString() : String {
        var s : String = "Sweep:\nlocalCenter: " + localCenter + "\n";
        s += "c0: " + c0 + ", c: " + c + "\n";
        s += "a0: " + a0 + ", a: " + a + "\n";
        s += "alpha0: " + alpha0;
        return s;
    }

    public function normalize() : Void {
        var d : Float = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI);
        a0 -= d;
        a -= d;
    }

    public function set(other : Sweep) : Sweep {
        localCenter.setVec(other.localCenter);
        c0.setVec(other.c0);
        c.setVec(other.c);
        a0 = other.a0;
        a = other.a;
        alpha0 = other.alpha0;
        return this;
    }

    /**
     * Get the interpolated transform at a specific time.
     * 
     * @param xf the result is placed here - must not be null
     * @param t the normalized time in [0,1].
     */
     public function getTransform(xf : Transform, beta : Float) : Void {
        // assert (xf != null);
        xf.p.x = (1.0 - beta) * c0.x + beta * c.x;
        xf.p.y = (1.0 - beta) * c0.y + beta * c.y;
        var angle : Float = (1.0 - beta) * a0 + beta * a;
        xf.q.set(angle);
        // Shift to origin
        var q : Rot = xf.q;
        xf.p.x -= q.c * localCenter.x - q.s * localCenter.y;
        xf.p.y -= q.s * localCenter.x + q.c * localCenter.y;
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     * 
     * @param alpha the new initial time.
     */
     public function advance(alpha : Float) : Void {
        // assert(alpha0 < 1.0f);
        var beta : Float = (alpha - alpha0) / (1.0 - alpha0);
        c0.x += beta * (c.x - c0.x);
        c0.y += beta * (c.y - c0.y);
        a0 += beta * (a - a0);
        alpha0 = alpha;
    }

}