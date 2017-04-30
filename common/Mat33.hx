package box2d.common;

import box2d.common.Vec3;

class Mat33 {

    private static var serialVersionUID : Int = 2;

    public static var IDENTITY : Mat33 = new Mat33(new Vec3().set(1, 0, 0), new Vec3().set(0, 1, 0), new Vec3().set(0, 0, 1));

    public var ex : Vec3;
    public var ey : Vec3;
    public var ez : Vec3;
    
    public function new(argCol1:Vec3=null, argCol2:Vec3=null, argCol3:Vec3=null) {
        if(argCol1!=null) {
            ex = argCol1.clone();
        } else {
            ex = new Vec3();
        }
        if(argCol2!=null) {
            ey = argCol2.clone();
        } else {
            ey = new Vec3();
        }
        if(argCol3!=null) {
            ez = argCol3.clone();
        } else {
            ez = new Vec3();
        }
    }

    public function setFloats (exx:Float, exy:Float, exz:Float, eyx:Float, eyy:Float, eyz:Float, ezx:Float, ezy:Float, ezz:Float) : Void {
        ex = new Vec3().set(exx, exy, exz);
        ey = new Vec3().set(eyx, eyy, eyz);
        ez = new Vec3().set(ezx, ezy, ezz);
    }

     public function setZero() : Void {
        ex.setZero();
        ey.setZero();
        ez.setZero();
    }

    public function set(exx:Float, exy:Float, exz:Float, eyx:Float, eyy:Float, eyz:Float, ezx:Float, ezy:Float, ezz:Float) : Void {
        ex.x = exx;
        ex.y = exy;
        ex.z = exz;
        ey.x = eyx;
        ey.y = eyy;
        ey.z = eyz;
        ez.x = eyx;
        ez.y = eyy;
        ez.z = eyz;
    }

    public function setMat(mat:Mat33) : Void {
        var vec : Vec3 = mat.ex;
        ex.x = vec.x;
        ex.y = vec.y;
        ex.z = vec.z;
        var vec1 : Vec3 = mat.ey;
        ey.x = vec1.x;
        ey.y = vec1.y;
        ey.z = vec1.z;
        var vec2 : Vec3 = mat.ez;
        ez.x = vec2.x;
        ez.y = vec2.y;
        ez.z = vec2.z;
    }

    public function setIdentity() : Void {
        ex.x = 1;
        ex.y = 0;
        ex.z = 0;
        ey.x = 0;
        ey.y = 1;
        ey.z = 0;
        ez.x = 0;
        ez.y = 0;
        ez.z = 1;
    }

    // / Multiply a matrix times a vector.
    public static function mul(A : Mat33, v : Vec3) : Vec3 {
        return new Vec3().set(v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x, v.x * A.ex.y + v.y * A.ey.y + v.z
            * A.ez.y, v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z);
    }

    public static function mul22(A : Mat33, v : Vec2) : Vec2 {
        return new Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
    }

    public static function mul22ToOut(A : Mat33, v : Vec2, out : Vec2) : Void {
        var tempx : Float = A.ex.x * v.x + A.ey.x * v.y;
        out.y = A.ex.y * v.x + A.ey.y * v.y;
        out.x = tempx;
    }

    public static function mul22ToOutUnsafe(A : Mat33, v : Vec2, out : Vec2) : Void {
        // assert (v != out);
        out.y = A.ex.y * v.x + A.ey.y * v.y;
        out.x = A.ex.x * v.x + A.ey.x * v.y;
    }

    public static function mulToOut(A : Mat33, v : Vec3, out : Vec3) : Void {
        var tempy : Float = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
        var tempz : Float = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
        out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
        out.y = tempy;
        out.z = tempz;
    }

    public static function mulToOutUnsafe(A : Mat33, v : Vec3, out : Vec3) : Void {
        // assert (out != v);
        out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
        out.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
        out.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
     * in one-shot cases.
     * 
     * @param b
     * @return
     */
     public function solve22(b : Vec2) : Vec2 {
        var x : Vec2 = new Vec2();
        solve22ToOut(b, x);
        return x;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
     * in one-shot cases.
     * 
     * @param b
     * @return
     */
     public function solve22ToOut(b : Vec2, out : Vec2) : Void {
        var a11 : Float = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
        var det : Float = a11 * a22 - a12 * a21;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        out.x = det * (a22 * b.x - a12 * b.y);
        out.y = det * (a11 * b.y - a21 * b.x);
    }

    // djm pooling from below
    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
     * in one-shot cases.
     * 
     * @param b
     * @return
     */
    public function solve33(b:Vec3) : Vec3 {
        var x : Vec3 = new Vec3();
        solve33ToOut(b, x);
        return x;
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
     * in one-shot cases.
     * 
     * @param b
     * @param out the result
     */
     public function solve33ToOut(b : Vec3, out : Vec3) : Void {
        // assert (b != out);
        Vec3.crossToOutUnsafe(ey, ez, out);
        var det : Float = Vec3.dot(ex, out);
        if (det != 0.0) {
            det = 1.0 / det;
        }
        Vec3.crossToOutUnsafe(ey, ez, out);
        var x : Float = det * Vec3.dot(b, out);
        Vec3.crossToOutUnsafe(b, ez, out);
        var y : Float = det * Vec3.dot(ex, out);
        Vec3.crossToOutUnsafe(ey, b, out);
        var z : Float = det * Vec3.dot(ex, out);
        out.x = x;
        out.y = y;
        out.z = z;
    }

     public function getInverse22(M : Mat33) : Void {
        var a : Float = ex.x, b = ey.x, c = ex.y, d = ey.y;
        var det : Float = a * d - b * c;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0.0;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0.0;
        M.ez.x = 0.0;
        M.ez.y = 0.0;
        M.ez.z = 0.0;
    }

    // / Returns the zero matrix if singular.
    public function getSymInverse33(M : Mat33) : Void {
        var bx : Float = ey.y * ez.z - ey.z * ez.y;
        var by : Float = ey.z * ez.x - ey.x * ez.z;
        var bz : Float = ey.x * ez.y - ey.y * ez.x;
        var det = ex.x * bx + ex.y * by + ex.z * bz;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var a11 : Float = ex.x, a12 = ey.x, a13 = ez.x;
        var a22 : Float = ey.y, a23 = ez.y;
        var a33 : Float = ez.z;
        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);
        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);
        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    }


    public static function setScaleTransform(scale : Float, out : Mat33) : Void {
        out.ex.x = scale;
        out.ey.y = scale;
    }

    public function hashCode() : Int {
        var prime : Int = 31;
        var result : Int = 1;
        result = prime * result + ((ex == null) ? 0 : ex.hashCode());
        result = prime * result + ((ey == null) ? 0 : ey.hashCode());
        result = prime * result + ((ez == null) ? 0 : ez.hashCode());
        return result;
    }

    public function equals(obj : Dynamic) : Bool {
        // if (this == obj) return true;
        // if (obj == null) return false;
        // if (getClass() != obj.getClass()) return false;
        // Mat33 other = (Mat33) obj;
        // if (ex == null) {
        // if (other.ex != null) return false;
        // } else if (!ex.equals(other.ex)) return false;
        // if (ey == null) {
        // if (other.ey != null) return false;
        // } else if (!ey.equals(other.ey)) return false;
        // if (ez == null) {
        // if (other.ez != null) return false;
        // } else if (!ez.equals(other.ez)) return false;
        return true;
    }

}