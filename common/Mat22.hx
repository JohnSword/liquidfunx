package box2d.common;

class Mat22 {

    private static var serialVersionUID : Int = 2L;

    public var ex:Vec2;
    public var ey:Vec2;

    public function new(c1:Vec2=null, c2:Vec2=null) {
        if(c1!=null) {
            this.ex = c1.clone();
        } else {
            this.ex = new Vec2();
        }
        if(c2!=null) {
            this.ey = c2.clone();
        } else {
            this.ey = new Vec2();
        }
    }

    /**
     * Create a matrix from four floats.
     * 
     * @param exx
     * @param col2x
     * @param exy
     * @param col2y
     */
    public function fromFloats(exx:Float, col2x:Float, exy:Float, col2y:Float) : Void {
        ex = new Vec2(exx, exy);
        ey = new Vec2(col2x, col2y);
    }

    /**
     * Set as a copy of another matrix.
     * 
     * @param m Matrix to copy
     */
    public function setMat22(m:Mat22) : Mat22 {
        ex.x = m.ex.x;
        ex.y = m.ex.y;
        ey.x = m.ey.x;
        ey.y = m.ey.y;
        return this;
    }

    public function set(exx:Float, col2x:Float, exy:Float, col2y:Float) : Mat22 {
        ex.x = exx;
        ex.y = exy;
        ey.x = col2x;
        ey.y = col2y;
        return this;
    }

    /**
     * Return a clone of this matrix. djm fixed double allocation
     */
    public function clone() : Mat22 {
        return new Mat22(ex, ey);
    }

    /**
     * Set as a matrix representing a rotation.
     * 
     * @param angle Rotation (in radians) that matrix represents.
     */
    public function setAngle(angle:Float) : Void {
        var c : Float = MathUtils.cos(angle);
        var s : Float = MathUtils.sin(angle);
        ex.x = c;
        ey.x = -s;
        ex.y = s;
        ey.y = c;
    }

    /**
     * Set as the identity matrix.
     */
    public function setIdentity() : Void {
        ex.x = 1.0;
        ey.x = 0.0;
        ex.y = 0.0;
        ey.y = 1.0;
    }

    /**
     * Set as the zero matrix.
     */
    public function setZero() : Void {
        ex.x = 0.0;
        ey.x = 0.0;
        ex.y = 0.0;
        ey.y = 0.0;
    }

    /**
     * Extract the angle from this matrix (assumed to be a rotation matrix).
     * 
     * @return
     */
    public function getAngle() : Float {
        return MathUtils.atan2(ex.y, ex.x);
    }

    /**
     * Set by column vectors.
     * 
     * @param c1 Column 1
     * @param c2 Column 2
     */
    public function setVec(c1:Vec2, c2:Vec2) : Void {
        ex.x = c1.x;
        ey.x = c2.x;
        ex.y = c1.y;
        ey.y = c2.y;
    }

    /** Returns the inverted Mat22 - does NOT invert the matrix locally! */
    public function invert() : Mat22 {
        var a : Float = ex.x, b = ey.x, c = ex.y, d = ey.y;
        var B : Mat22 = new Mat22();
        var det : Float = a * d - b * c;
        if (det != 0) {
            det = 1.0 / det;
        }
        B.ex.x = det * d;
        B.ey.x = -det * b;
        B.ex.y = -det * c;
        B.ey.y = det * a;
        return B;
    }

    public function invertLocal() : Mat22 {
        var a : Float = ex.x, b = ey.x, c = ex.y, d = ey.y;
        var det : Float = a * d - b * c;
        if (det != 0) {
            det = 1.0 / det;
        }
        ex.x = det * d;
        ey.x = -det * b;
        ex.y = -det * c;
        ey.y = det * a;
        return this;
    }

    public function invertToOut(out:Mat22) : Void {
        var a : Float = ex.x, b = ey.x, c = ex.y, d = ey.y;
        var det : Float = a * d - b * c;
        det = 1.0 / det;
        out.ex.x = det * d;
        out.ey.x = -det * b;
        out.ex.y = -det * c;
        out.ey.y = det * a;
    }

    /**
     * Return the matrix composed of the absolute values of all elements. djm: fixed double allocation
     * 
     * @return Absolute value matrix
     */
    public function abs() : Mat22 {
        var m : Mat22 = new Mat22();
        m.set(MathUtils.abs(ex.x), MathUtils.abs(ey.x), MathUtils.abs(ex.y), MathUtils.abs(ey.y));
        return m;
    }

    /* djm: added */
    public function absLocal() : Void {
        ex.absLocal();
        ey.absLocal();
    }

    /**
     * Return the matrix composed of the absolute values of all elements.
     * 
     * @return Absolute value matrix
     */
    public static function abs2(R:Mat22) : Mat22 {
        return R.abs();
    }

    /* djm created */
    public static function absToOut(R:Mat22, out:Mat22) : Void {
        out.ex.x = MathUtils.abs(R.ex.x);
        out.ex.y = MathUtils.abs(R.ex.y);
        out.ey.x = MathUtils.abs(R.ey.x);
        out.ey.y = MathUtils.abs(R.ey.y);
    }

    /**
     * Multiply a vector by this matrix.
     * 
     * @param v Vector to multiply by matrix.
     * @return Resulting vector
     */
    public function mulVec(v:Vec2) : Vec2 {
        return new Vec2(ex.x * v.x + ey.x * v.y, ex.y * v.x + ey.y * v.y);
    }

    public function mulToOutVec(v : Vec2, out : Vec2) : Void {
        var tempy : Float = ex.y * v.x + ey.y * v.y;
        out.x = ex.x * v.x + ey.x * v.y;
        out.y = tempy;
    }

    public function mulToOutUnsafeVec(v:Vec2, out:Vec2) : Void {
        // assert (v != out);
        out.x = ex.x * v.x + ey.x * v.y;
        out.y = ex.y * v.x + ey.y * v.y;
    }

    /**
     * Multiply another matrix by this one (this one on left). djm optimized
     * 
     * @param R
     * @return
     */
    public function mul(R:Mat22) : Mat22 {
        var C : Mat22 = new Mat22();
        C.ex.x = ex.x * R.ex.x + ey.x * R.ex.y;
        C.ex.y = ex.y * R.ex.x + ey.y * R.ex.y;
        C.ey.x = ex.x * R.ey.x + ey.x * R.ey.y;
        C.ey.y = ex.y * R.ey.x + ey.y * R.ey.y;
        return C;
    }

    public function mulLocal(R:Mat22) : Mat22 {
        mulToOut(R, this);
        return this;
    }

    public function mulToOut(R:Mat22, out:Mat22) : Void {
        var tempy1 : Float = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
        var tempx1 : Float = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
        out.ex.x = tempx1;
        out.ex.y = tempy1;
        var tempy2 : Float = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
        var tempx2 : Float = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
        out.ey.x = tempx2;
        out.ey.y = tempy2;
    }

    public function mulToOutUnsafe(R:Mat22, out:Mat22) : Void {
        // assert (out != R);
        // assert (out != this);
        out.ex.x = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
        out.ex.y = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
        out.ey.x = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
        out.ey.y = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
    }

    /**
     * Multiply another matrix by the transpose of this one (transpose of this one on left). djm:
     * optimized
     * 
     * @param B
     * @return
     */
    public function mulTrans(B:Mat22) : Mat22 {
        var C : Mat22 = new Mat22();

        C.ex.x = Vec2.dot(this.ex, B.ex);
        C.ex.y = Vec2.dot(this.ey, B.ex);

        C.ey.x = Vec2.dot(this.ex, B.ey);
        C.ey.y = Vec2.dot(this.ey, B.ey);
        return C;
    }

    public function mulTransLocal(B:Mat22) : Mat22 {
        mulTransToOut(B, this);
        return this;
    }

    public function mulTransToOut(B:Mat22, out:Mat22) : Void {
        var x1 : Float = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
        var y1 : Float = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
        var x2 : Float = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
        var y2 : Float = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
        out.ex.x = x1;
        out.ey.x = x2;
        out.ex.y = y1;
        out.ey.y = y2;
    }

    public function mulTransToOutUnsafe(B:Mat22, out:Mat22) : Void {
        // assert (B != out);
        // assert (this != out);
        out.ex.x = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
        out.ey.x = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
        out.ex.y = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
        out.ey.y = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
    }

    /**
     * Multiply a vector by the transpose of this matrix.
     * 
     * @param v
     * @return
     */
     public function mulTransVec(v: Vec2) : Vec2 {
        return new Vec2((v.x * ex.x + v.y * ex.y), (v.x * ey.x + v.y * ey.y));
    }

    /* djm added */
    public function mulTransToOutVec(v : Vec2, out : Vec2) : Void {
        var tempx : Float = v.x * ex.x + v.y * ex.y;
        out.y = v.x * ey.x + v.y * ey.y;
        out.x = tempx;
    }

    /**
     * Add this matrix to B, return the result.
     * 
     * @param B
     * @return
     */
     public function add(B : Mat22) : Mat22 {
        var m : Mat22 = new Mat22();
        m.ex.x = ex.x + B.ex.x;
        m.ex.y = ex.y + B.ex.y;
        m.ey.x = ey.x + B.ey.x;
        m.ey.y = ey.y + B.ey.y;
        return m;
    }

    /**
     * Add B to this matrix locally.
     * 
     * @param B
     * @return
     */
     public function addLocal(B : Mat22) : Mat22 {
        ex.x += B.ex.x;
        ex.y += B.ex.y;
        ey.x += B.ey.x;
        ey.y += B.ey.y;
        return this;
    }

    /**
     * Solve A * x = b where A = this matrix.
     * 
     * @return The vector x that solves the above equation.
     */
     public function solve(b : Vec2) : Vec2 {
        var a11 : Float = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
        var det : Float = a11 * a22 - a12 * a21;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var x : Vec2 = new Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
        return x;
    }

    public function solveToOut(b : Vec2, out : Vec2) : Void {
        var a11 : Float = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
        var det : Float = a11 * a22 - a12 * a21;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var tempy : Float = det * (a11 * b.y - a21 * b.x);
        out.x = det * (a22 * b.x - a12 * b.y);
        out.y = tempy;
    }

    public static function mulMV(R : Mat22, v : Vec2) : Vec2 {
        return new Vec2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y);
    }

    public static function mulToOutMVV(R : Mat22, v : Vec2, out : Vec2) : Void {
        var tempy : Float = R.ex.y * v.x + R.ey.y * v.y;
        out.x = R.ex.x * v.x + R.ey.x * v.y;
        out.y = tempy;
    }

    public static function mulToOutUnsafeMVV(R : Mat22, v : Vec2, out : Vec2) : Void {
        // assert (v != out);
        out.x = R.ex.x * v.x + R.ey.x * v.y;
        out.y = R.ex.y * v.x + R.ey.y * v.y;
    }

     public static function mul2(A : Mat22, B : Mat22) : Mat22 {
        var C : Mat22 = new Mat22();
        C.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
        C.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
        C.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
        C.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
        return C;
    }

    public static function mulToOut2(A : Mat22, B : Mat22, out : Mat22) : Void {
        var tempy1 : Float = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
        var tempx1 : Float = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
        var tempy2 : Float = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
        var tempx2 : Float = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
        out.ex.x = tempx1;
        out.ex.y = tempy1;
        out.ey.x = tempx2;
        out.ey.y = tempy2;
    }

    public static function mulToOutUnsafe2(A : Mat22, B : Mat22, out : Mat22) : Void {
        // assert (out != A);
        // assert (out != B);
        out.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
        out.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
        out.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
        out.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
    }

    public static function mulTrans2(R : Mat22, v : Vec2) : Vec2 {
        return new Vec2((v.x * R.ex.x + v.y * R.ex.y), (v.x * R.ey.x + v.y * R.ey.y));
    }

    public static function mulTransToOut2(R : Mat22, v : Vec2, out : Vec2) : Void {
        var outx : Float = v.x * R.ex.x + v.y * R.ex.y;
        out.y = v.x * R.ey.x + v.y * R.ey.y;
        out.x = outx;
    }

    public static function mulTransToOutUnsafe2(R : Mat22, v : Vec2, out : Vec2) : Void {
        // assert (out != v);
        out.y = v.x * R.ey.x + v.y * R.ey.y;
        out.x = v.x * R.ex.x + v.y * R.ex.y;
    }

    public static function Mat22 mulTrans2(A : Mat22, B : Mat22) {
        var C : Mat22 = new Mat22();
        C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
        C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
        C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
        C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
        return C;
    }

    public static function mulTransToOut2MMM(A : Mat22, B : Mat22, out : Mat22) : Void {
        var x1 : Float = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
        var y1 : Float = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
        var x2 : Float = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
        var y2 : Float = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
        out.ex.x = x1;
        out.ex.y = y1;
        out.ey.x = x2;
        out.ey.y = y2;
    }

    public static function mulTransToOutUnsafe2MMM(A : Mat22, B : Mat22, out : Mat22) : Void {
        // assert (A != out);
        // assert (B != out);
        out.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
        out.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
        out.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
        out.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
    }

    public static function createRotationalTransform(angle : Float) : Mat22 {
        var mat : Mat22 = new Mat22();
        var c : Float = MathUtils.cos(angle);
        var s : Float = MathUtils.sin(angle);
        mat.ex.x = c;
        mat.ey.x = -s;
        mat.ex.y = s;
        mat.ey.y = c;
        return mat;
    }

    public static function createRotationalTransform2(angle : Float, out : Mat22) : Void {
        var c : Float = MathUtils.cos(angle);
        var s : Float = MathUtils.sin(angle);
        out.ex.x = c;
        out.ey.x = -s;
        out.ex.y = s;
        out.ey.y = c;
    }

    public static function createScaleTransform(scale : Float) : Mat22 {
        var mat : Mat22 = new Mat22();
        mat.ex.x = scale;
        mat.ey.y = scale;
        return mat;
    }

    public static function createScaleTransformFM(scale : Float, out : Mat22) : Void {
        out.ex.x = scale;
        out.ey.y = scale;
    }

    public function equals(Object obj): Bool{
        // TODO: equals not implemented
        // if (this == obj) return true;
        // if (obj == null) return false;
        // if (getClass() != obj.getClass()) return false;
        // var other : Mat22 = (Mat22) obj;
        // if (ex == null) {
        // if (other.ex != null) return false;
        // } else if (!ex.equals(other.ex)) return false;
        // if (ey == null) {
        // if (other.ey != null) return false;
        // } else if (!ey.equals(other.ey)) return false;
        return true;
    }

}