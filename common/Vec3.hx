package box2d.common;

class Vec3 {

    private static var serialVersionUID : Int = 1;

    public var x : Float;
    public var y : Float;
    public var z : Float;
    
    public function new() {
        x = y = z = 0;
    }

     public function setVec(vec : Vec3) : Vec3 {
        x = vec.x;
        y = vec.y;
        z = vec.z;
        return this;
    }

    public function set(argX : Float, argY : Float, argZ : Float) : Vec3 {
        x = argX;
        y = argY;
        z = argZ;
        return this;
    }

    public function addLocal(argVec: Vec3) : Vec3 {
        x += argVec.x;
        y += argVec.y;
        z += argVec.z;
        return this;
    }

    public function add(argVec: Vec3) : Vec3 {
        var v = new Vec3();
        v.setVec(argVec);
        return v;
    }

    public function subLocal(argVec: Vec3) : Vec3 {
        x -= argVec.x;
        y -= argVec.y;
        z -= argVec.z;
        return this;
    }

    public function sub(argVec: Vec3) : Vec3 {
        var v : Vec3 = new Vec3();
        v.x = x - argVec.x;
        v.y = y - argVec.y;
        v.z = z - argVec.z;
        return v;
    }

    public function mulLocal(argScalar:Float) : Vec3 {
        x *= argScalar;
        y *= argScalar;
        z *= argScalar;
        return this;
    }

    public function mul(argScalar:Float) : Vec3 {
        var v : Vec3 = new Vec3();
        v.x = x * argScalar;
        v.y = y * argScalar;
        v.z = z * argScalar;
        return v;
    }

    public function negate() : Vec3 {
        var v : Vec3 = new Vec3();
        v.x = -x;
        v.y = -y;
        v.z = -z;
        return v;
    }

    public function negateLocal() : Vec3 {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    public function setZero() : Void {
        x = 0;
        y = 0;
        z = 0;
    }

    public function clone() : Vec3 {
        var v : Vec3 = new Vec3();
        v.setVec(this);
        return v;
    }

    public function toString() : String {
        return "(" + x + "," + y + "," + z + ")";
    }

    public function hashCode() : Int {
        var prime : Int = 31;
        var result : Int = 1;
        result = prime * result + Std.int(x);
        result = prime * result + Std.int(y);
        result = prime * result + Std.int(z);
        // result = prime * result + Float.floatToIntBits(x);
        // result = prime * result + Float.floatToIntBits(y);
        // result = prime * result + Float.floatToIntBits(z);
        return result;
    }

    public function equals(obj : Dynamic) : Bool {
        // TODO: unimplemented?
        if (this == obj) return true;
        if (obj == null) return false;
        if(Type.getClass(this) != Type.getClass(obj)) return false;
        // if (getClass() != obj.getClass()) return false;
        var other : Vec3 = cast obj;
        if (Std.int(x) != Std.int(other.x)) return false;
        if (Std.int(y) != Std.int(other.x)) return false;
        if (Std.int(z) != Std.int(other.x)) return false;
        // if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) return false;
        // if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) return false;
        // if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z)) return false;
        return true;
    }

    public static function dot(a: Vec3, b: Vec3) : Float {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    public static function cross(a: Vec3, b: Vec3) : Vec3 {
        var v : Vec3 = new Vec3();
        v.x = a.y * b.z - a.z * b.y;
        v.y = a.z * b.x - a.x * b.z;
        v.z = a.x * b.y - a.y * b.x;
        return v;
    }

    public static function crossToOut(a: Vec3, b: Vec3, out: Vec3) : Void {
        var tempy : Float = a.z * b.x - a.x * b.z;
        var tempz : Float = a.x * b.y - a.y * b.x;
        out.x = a.y * b.z - a.z * b.y;
        out.y = tempy;
        out.z = tempz;
    }
    
    public static function crossToOutUnsafe(a: Vec3, b: Vec3, out: Vec3) : Void {
        out.x = a.y * b.z - a.z * b.y;
        out.y = a.z * b.x - a.x * b.z;
        out.z = a.x * b.y - a.y * b.x;
    }

}