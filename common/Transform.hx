package box2d.common;

class Transform {

    private static var serialVersionUID : Int = 1L;

    /** The translation caused by the transform */
    public var p : Vec2;

    /** A matrix representing a rotation */
    public var q : Rot;

    private static var pool : Vec2 = new Vec2();

    public function new(xf : Transform = null) {
        if(xf != null) {
            p = xf.p.clone();
            q = xf.q.clone();
        } else {
            p = new Vec2();
            q = new Rot();
        }
    }

    // public Transform(final Vec2 _position, final Rot _R) {
    //     p = _position.clone();
    //     q = _R.clone();
    // }

    /** Set this to equal another transform. */
    public function set(xf : Transform) : Transform {
        p.setVec(xf.p);
        q.setRot(xf.q);
        return this;
    }

    /**
     * Set this based on the position and angle.
     * 
     * @param p
     * @param angle
     */
     public function setVF(p : Vec2, angle : Float) : Void {
        this.p.setVec(p);
        q.set(angle);
    }

    /** Set this to the identity transform. */
    public function setIdentity() : Void {
        p.setZero();
        q.setIdentity();
    }

    public static function mul(T : Transform, v : Vec2) : Vec2 {
        return new Vec2((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y);
    }

    public static function mulToOut(T : Transform, v : Vec2, out : Vec2) : Void {
        var tempy : Float = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
        out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        out.y = tempy;
    }

    public static function mulToOutUnsafe(T : Transform, v : Vec2, out  : Vec2) : Void {
        // assert (v != out);
        out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        out.y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
    }

    public static function mulTrans(T : Transform, v : Vec2) : Vec2 {
        var px : Float = v.x - T.p.x;
        var py : Float = v.y - T.p.y;
        return new Vec2((T.q.c * px + T.q.s * py), (-T.q.s * px + T.q.c * py));
    }

    public static function mulTransToOut(T : Transform, v : Vec2, out : Vec2) : Void {
        var px : Float = v.x - T.p.x;
        var py : Float = v.y - T.p.y;
        var tempy : Float = (-T.q.s * px + T.q.c * py);
        out.x = (T.q.c * px + T.q.s * py);
        out.y = tempy;
    }
    
    public static function mulTransToOutUnsafe(T : Transform, v : Vec2, out : Vec2) : Void {
        // assert(v != out);
        var px : Float = v.x - T.p.x;
        var py : Float = v.y - T.p.y;
        out.x = (T.q.c * px + T.q.s * py);
        out.y = (-T.q.s * px + T.q.c * py);
    }

    public static function mul2(A : Transform, B : Transform) {
        var C : Transform = new Transform();
        Rot.mulUnsafe(A.q, B.q, C.q);
        Rot.mulToOutUnsafe(A.q, B.p, C.p);
        C.p.addLocalVec(A.p);
        return C;
    }

    public static function mulToOut2(A : Transform, B : Transform, out : Transform) : Void {
        // assert (out != A);
        Rot.mul(A.q, B.q, out.q);
        Rot.mulToOut(A.q, B.p, out.p);
        out.p.addLocalVec(A.p);
    }

    public static function mulToOutUnsafe2(A : Transform, B : Transform, out : Transform) : Void {
        // assert (out != B);
        // assert (out != A);
        Rot.mulUnsafe(A.q, B.q, out.q);
        Rot.mulToOutUnsafe(A.q, B.p, out.p);
        out.p.addLocalVec(A.p);
    }

    public static function Transform mulTrans(A : Transform, B : Transform) {
        Transform C = new Transform();
        Rot.mulTransUnsafe(A.q, B.q, C.q);
        pool.set(B.p).subLocal(A.p);
        Rot.mulTransUnsafe(A.q, pool, C.p);
        return C;
    }

    public static function mulTransToOut2(A : Transform, B : Transform, out : Transform) : Void {
        // assert (out != A);
        Rot.mulTrans(A.q, B.q, out.q);
        pool.setVec(B.p).subLocal(A.p);
        Rot.mulTrans2(A.q, pool, out.p);
    }

    public static function mulTransToOutUnsafe2(A : Transform, B : Transform, out : Transform) : Void {
        // assert (out != A);
        // assert (out != B);
        Rot.mulTransUnsafe(A.q, B.q, out.q);
        pool.setVec(B.p).subLocal(A.p);
        Rot.mulTransUnsafe2(A.q, pool, out.p);
    }

    public function toString() : String {
        var s : String = "XForm:\n";
        s += "Position: " + p + "\n";
        s += "R: \n" + q + "\n";
        return s;
    }

}