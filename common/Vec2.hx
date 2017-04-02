package box2d.common;

class Vec2 {

    private static final long serialVersionUID = 1L;

    public var x:Float;
    public var y:Float;

    public function new(x:Float=0,y:Float=0) {
        this.x = x;
        this.y = y;
    }

    /** Zero out this vector. */
    public function setZero() : Void {
        this.x = 0.0;
        this.y = 0.0;
    }

    /** Set the vector component-wise. */
    public function set(x:Float, y:Float) : Vec2 {
        this.x = x;
        this.y = y;
        return this;
    }

    /** Set this vector to another vector. */
    public function setVec(v:Vec2) : Vec2 {
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    /** Return the sum of this vector and another; does not alter either one. */
    public function add(v:Vec2) : Vec2 {
        return new Vec2(this.x + v.x, this.y + v.y);
    }

    /** Return the difference of this vector and another; does not alter either one. */
    public function sub(v:Vec2) : Vec2 {
        return new Vec2(this.x - v.x, this.y - v.y);
    }

    /** Return this vector multiplied by a scalar; does not alter this vector. */
    public function Vec2 mul(a:Float) {
        return new Vec2(this.x * a, this.y * a);
    }

    /** Return the negation of this vector; does not alter this vector. */
    public function negate() : Vec2 {
        return new Vec2(-x, -y);
    }

    /** Flip the vector and return it - alters this vector. */
    public function negateLocal() : Vec2 {
        x = -x;
        y = -y;
        return this;
    }

    /** Add another vector to this one and returns result - alters this vector. */
    public function addLocalVec(v:Vec2) : Vec2 {
        x += v.x;
        y += v.y;
        return this;
    }

    /** Adds values to this vector and returns result - alters this vector. */
    public function addLocal(x:Float, y:Float) : Vec2 {
        this.x += x;
        this.y += y;
        return this;
    }

  /** Subtract another vector from this one and return result - alters this vector. */
  public function subLocal(v:Vec2) : Vec2 {
    x -= v.x;
    y -= v.y;
    return this;
  }

  /** Multiply this vector by a number and return result - alters this vector. */
  public function mulLocal(a:Float) : Vec2 {
    x *= a;
    y *= a;
    return this;
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  public function skew() : Vec2 {
    return new Vec2(-y, x);
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  public function skewVec(out:Vec2) : Void {
    out.x = -y;
    out.y = x;
  }

  /** Return the length of this vector. */
  public function length() : Float {
    return MathUtils.sqrt(x * x + y * y);
  }

  /** Return the squared length of this vector. */
  public function lengthSquared() : Float {
    return (x * x + y * y);
  }

  /** Normalize this vector and return the length before normalization. Alters this vector. */
  public function normalize() : Float {
    var length : Float = length();
    if (length < Settings.EPSILON) {
      return 0.0;
    }

    var invLength : Float = 1.0 / length;
    x *= invLength;
    y *= invLength;
    return length;
  }

  /** True if the vector represents a pair of valid, non-infinite floating point numbers. */
  public function isValid() : bool {
    return !Float.isNaN(x) && !Float.isInfinite(x) && !Float.isNaN(y) && !Float.isInfinite(y);
  }

  /** Return a new vector that has positive components. */
  public function abs() : Vec2 {
    return new Vec2(MathUtils.abs(x), MathUtils.abs(y));
  }

  public function absLocal() : Void {
    x = MathUtils.abs(x);
    y = MathUtils.abs(y);
  }

  /** Return a copy of this vector. */
  public function clone() : Vec2 {
    return new Vec2(x, y);
  }

  public function toString() : String {
    return "(" + x + "," + y + ")";
  }

  /*
   * Static
   */

  public static function abs2(a : Vec2) : Vec2 {
    return new Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y));
  }

  public static function absToOut(a : Vec2, out : Vec2) : Void {
    out.x = MathUtils.abs(a.x);
    out.y = MathUtils.abs(a.y);
  }

  public static function dot(a : Vec2, b : Vec2) : Float {
    return a.x * b.x + a.y * b.y;
  }

  public static function crossVec(a : Vec2, b : Vec2) : Float {
    return a.x * b.y - a.y * b.x;
  }

  public static function cross(a : Vec2, s : Float) : Vec2 {
    return new Vec2(s * a.y, -s * a.x);
  }

  public static function crossToOut(a : Vec2, s : Float, out : Vec2) {
    var tempy : Float = -s * a.x;
    out.x = s * a.y;
    out.y = tempy;
  }

  public static function crossToOutUnsafe(a : Vec2, s : Float, out : Vec2) {
    // assert (out != a);
    out.x = s * a.y;
    out.y = -s * a.x;
  }

  public static function crossFV(s : Float, a : Vec2) : Vec2 {
    return new Vec2(-s * a.y, s * a.x);
  }

  public static function crossToOutFVV(s : Float, a : Vec2, out : Vec2) : Void {
    var tempY : Float = s * a.x;
    out.x = -s * a.y;
    out.y = tempY;
  }

  public static function crossToOutUnsafeFVV(s : Float, a : Vec2, out : Vec2) : Void {
    // assert (out != a);
    out.x = -s * a.y;
    out.y = s * a.x;
  }

  public static function negateToOut(a : Vec2, out : Vec2) : Void {
    out.x = -a.x;
    out.y = -a.y;
  }

  public static function min(a : Vec2, b : Vec2) : Vec2 {
    return new Vec2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
  }

  public static function max(a : Vec2, b : Vec2) : Vec2 {
    return new Vec2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
  }

  public static function minToOut(a : Vec2, b : Vec2, out : Vec2) : Void {
    out.x = a.x < b.x ? a.x : b.x;
    out.y = a.y < b.y ? a.y : b.y;
  }

  public static function maxToOut(a : Vec2, b : Vec2, out : Vec2) : Void {
    out.x = a.x > b.x ? a.x : b.x;
    out.y = a.y > b.y ? a.y : b.y;
  }

  /**
   * @see java.lang.Object#equals(java.lang.Object)
   */
  public function equals(obj:Dynamic) : Bool { // automatically generated by Eclipse
  // TODO: equals not implemented
    // if (this == obj) return true;
    // if (obj == null) return false;
    // if (getClass() != obj.getClass()) return false;
    // Vec2 other = (Vec2) obj;
    // if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) return false;
    // if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) return false;
    return true;
  }

}