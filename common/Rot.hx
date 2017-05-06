package box2d.common;

class Rot {

    private static var serialVersionUID : Int = 1;

    // sin and cos
    public var s : Float = 0;
    public var c : Float = 0;

    public function new(angle:Float=null) {
        if(angle!=null) {
            set(angle);
        } else {
            setIdentity();
        }
    }

    public function getSin() : Float {
        return s;
    }

    public function toString() : String {
        return "Rot(s:" + s + ", c:" + c + ")";
    }

    public function getCos() : Float {
        return c;
    }

    public function set(angle : Float) : Rot {
        s = MathUtils.sin(angle);
        c = MathUtils.cos(angle);
        return this;
    }

    public function setRot(other : Rot) : Rot {
        s = other.s;
        c = other.c;
        return this;
    }

    public function setIdentity() : Rot {
        s = 0;
        c = 1;
        return this;
    }

    public function getAngle() : Float {
        return MathUtils.atan2(s, c);
    }

    public function getXAxis(xAxis : Vec2) : Void {
        xAxis.set(c, s);
    }

    public function getYAxis(yAxis : Vec2) : Void {
        yAxis.set(-s, c);
    }

    public function clone() : Rot {
        var copy : Rot = new Rot();
        copy.s = s;
        copy.c = c;
        return copy;
    }

    public static function mul(q : Rot, r : Rot, out : Rot) : Void {
        var tempc : Float = q.c * r.c - q.s * r.s;
        out.s = q.s * r.c + q.c * r.s;
        out.c = tempc;
    }

    public static function mulUnsafe(q : Rot, r : Rot, out : Rot) : Void {
        // assert (r != out);
        // assert (q != out);
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs qc] [rs rc] [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        out.s = q.s * r.c + q.c * r.s;
        out.c = q.c * r.c - q.s * r.s;
    }

    public static function mulTrans(q : Rot, r : Rot, out : Rot) : Void {
        var tempc : Float = q.c * r.c + q.s * r.s;
        out.s = q.c * r.s - q.s * r.c;
        out.c = tempc;
    }

    public static function mulTransUnsafe(q : Rot, r : Rot, out : Rot) : Void {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc] [rs rc] [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        out.s = q.c * r.s - q.s * r.c;
        out.c = q.c * r.c + q.s * r.s;
    }

    public static function mulToOut(q : Rot, v : Vec2, out : Vec2) : Void {
        var tempy : Float = q.s * v.x + q.c * v.y;
        out.x = q.c * v.x - q.s * v.y;
        out.y = tempy;
    }

    public static function mulToOutUnsafe(q : Rot, v : Vec2, out : Vec2) : Void {
        out.x = q.c * v.x - q.s * v.y;
        out.y = q.s * v.x + q.c * v.y;
    }

    public static function mulTrans2(q : Rot, v : Vec2, out : Vec2) : Void {
        var tempy : Float = -q.s * v.x + q.c * v.y;
        out.x = q.c * v.x + q.s * v.y;
        out.y = tempy;
    }

    public static function mulTransUnsafe2(q : Rot, v : Vec2, out : Vec2) : Void {
        out.x = q.c * v.x + q.s * v.y;
        out.y = -q.s * v.x + q.c * v.y;
    }

}