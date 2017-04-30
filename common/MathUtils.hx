package box2d.common;

class MathUtils extends PlatformMathUtils {

    public static var PI : Float = Math.PI;
    public static var TWOPI : Float = (Math.PI * 2);
    public static var INV_PI : Float = 1 / PI;
    public static var HALF_PI : Float = PI / 2;
    public static var QUARTER_PI : Float = PI / 4;
    public static var THREE_HALVES_PI : Float = TWOPI - HALF_PI;

    /**
    * Degrees to radians conversion factor
    */
    public static var DEG2RAD : Float = PI / 180;

    /**
    * Radians to degrees conversion factor
    */
    public static var RAD2DEG : Float = 180 / PI;

    public static var sinLUT : Array<Float> = new Array<Float>();

    // TODO: some sort of static stuff
    // static {
    //     for (int i = 0; i < Settings.SINCOS_LUT_LENGTH; i++) {
    //     sinLUT[i] = (float) Math.sin(i * Settings.SINCOS_LUT_PRECISION);
    //     }
    // }

    public static function sin(x : Float) : Float {
        if (Settings.SINCOS_LUT_ENABLED) {
            return sinLUTf(x);
        } else {
            return Math.sin(x);
        }
    }

    public static function sinLUTf(x : Float) : Float {
        x %= TWOPI;

        if (x < 0) {
        x += TWOPI;
        }

        if (Settings.SINCOS_LUT_LERP) {

        x /= Settings.SINCOS_LUT_PRECISION;

        // TODO: cast to int
        var index : Int = Std.int(x);

        if (index != 0) {
            x %= index;
        }

        // the next index is 0
        if (index == Settings.SINCOS_LUT_LENGTH - 1) {
            return ((1 - x) * sinLUT[index] + x * sinLUT[0]);
        } else {
            return ((1 - x) * sinLUT[index] + x * sinLUT[index + 1]);
        }

        } else {
        return sinLUT[MathUtils.round(x / Settings.SINCOS_LUT_PRECISION) % Settings.SINCOS_LUT_LENGTH];
        }
    }

    public static function cos(x:Float) : Float {
        if (Settings.SINCOS_LUT_ENABLED) {
            return sinLUTf(HALF_PI - x);
        } else {
            return Math.cos(x);
        }
    }

    // public static function abs(x:Float) : Float {
    // }

    public static function abs(x:Dynamic) : Dynamic {
        if(Std.is(x,Float)) {
            if (Settings.FAST_ABS) {
                return x > 0 ? x : -x;
            } else {
                return Math.abs(x);
            }
        } else {
            var y : Int = x >> 31;
            return (x ^ y) - y;
        } 
    }

    public static function fastAbs(x:Float) : Float {
        return x > 0 ? x : -x;
    }

    public static function floor(x:Float) : Int {
        if (Settings.FAST_FLOOR) {
            return fastFloor(x);
        } else {
            return Std.int(Math.floor(x));
        }
    }

    public static function fastFloor(x:Float) : Int {
        var y : Int = Std.int(x);
        if (x < y) {
            return y - 1;
        }
        return y;
    }

    public static function ceil(x:Float) : Int {
        if (Settings.FAST_CEIL) {
            return fastCeil(x);
        } else {
            return Std.int(Math.ceil(x));
        }
    }

    public static function fastCeil(x:Float) : Int {
        var y : Int = Std.int(x);
        if (x > y) {
            return y + 1;
        }
        return y;
    }

    public static function round(x:Float) : Int {
        if (Settings.FAST_ROUND) {
            return floor(x + .5);
        } else {
            return Math.round(x);
        }
    }

    /**
     * Rounds up the value to the nearest higher power^2 value.
     * 
     * @param x
     * @return power^2 value
     */
     public static function ceilPowerOf2(x:Int) : Int {
        var pow2 : Int = 1;
        while (pow2 < x) {
            pow2 <<= 1;
        }
        return pow2;
    }

    public static function max(a:Dynamic, b:Dynamic) : Dynamic {
        return a > b ? a : b;
    }

    // public static function int max(final int a, final int b) {
    //     return a > b ? a : b;
    // }

    public static function min(a:Dynamic, b:Dynamic) : Dynamic {
        return a < b ? a : b;
    }

    // public final static int min(final int a, final int b) {
    //     return a < b ? a : b;
    // }

    public static function map(val:Float, fromMin:Float, fromMax:Float, toMin:Float, toMax:Float) : Float {
        var mult : Float = (val - fromMin) / (fromMax - fromMin);
        var res : Float = toMin + mult * (toMax - toMin);
        return res;
    }

    /** Returns the closest value to 'a' that is in between 'low' and 'high' */
    public static function clamp(a:Float, low:Float, high:Float) : Float {
        return max(low, min(a, high));
    }

    public static function clampVec(a:Vec2, low:Vec2, high:Vec2) : Vec2 {
        var min : Vec2 = new Vec2();
        min.x = a.x < high.x ? a.x : high.x;
        min.y = a.y < high.y ? a.y : high.y;
        min.x = low.x > min.x ? low.x : min.x;
        min.y = low.y > min.y ? low.y : min.y;
        return min;
    }

    public static function clampToOut(a:Vec2, low:Vec2, high:Vec2, dest:Vec2) : Void {
        dest.x = a.x < high.x ? a.x : high.x;
        dest.y = a.y < high.y ? a.y : high.y;
        dest.x = low.x > dest.x ? low.x : dest.x;
        dest.y = low.y > dest.y ? low.y : dest.y;
    }

    /**
     * Next Largest Power of 2: Given a binary integer value x, the next largest power of 2 can be
     * computed by a SWAR algorithm that recursively "folds" the upper bits into the lower bits. This
     * process yields a bit vector with the same most significant 1 as x, but all 1's below it. Adding
     * 1 to that value yields the next largest power of 2.
     */
     public static function nextPowerOfTwo(x:Int) : Int {
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x + 1;
    }

    public static function isPowerOfTwo(x:Int) : Bool {
        return x > 0 && (x & x - 1) == 0;
    }

    public static function pow(a:Float, b:Float) : Float {
        // TODO: fast pow function call
        // if (Settings.FAST_POW) {
        //     return MathUtils.fastPow(a, b);
        // }
        return Math.pow(a, b);
    }

    public static function atan2(y:Float, x:Float) :Float {
        if (Settings.FAST_ATAN2) {
            return fastAtan2(y, x);
        } else {
            return Math.atan2(y, x);
        }
    }

    public static function fastAtan2(y:Float, x:Float) :Float {
        if (x == 0.0) {
            if (y > 0.0) return HALF_PI;
            if (y == 0.0) return 0.0;
            return -HALF_PI;
        }
        var atan : Float;
        var z : Float = y / x;
        if (abs(z) < 1.0) {
            atan = z / (1.0 + 0.28 * z * z);
            if (x < 0.0) {
                if (y < 0.0) return atan - PI;
                return atan + PI;
            }
        } else {
            atan = HALF_PI - z / (z * z + 0.28);
            if (y < 0.0) return atan - PI;
        }
        return atan;
    }

    public static function reduceAngle(theta:Float) : Float {
        theta %= TWOPI;
        if (abs(theta) > PI) {
            theta = theta - TWOPI;
        }
        if (abs(theta) > HALF_PI) {
            theta = PI - theta;
        }
        return theta;
    }

    public static function randomFloat(argLow:Float, argHigh:Float) : Float {
        return Math.random() * (argHigh - argLow) + argLow;
    }

    public static function randomFloat2(r:Random, argLow:Float, argHigh:Float) {
        return r.nextFloat() * (argHigh - argLow) + argLow;
    }

    public static function sqrt(x:Float) :Float {
        return Math.sqrt(x);
    }

    public static function distanceSquared(v1:Vec2, v2:Vec2) :Float {
        var dx:Float = (v1.x - v2.x);
        var dy:Float = (v1.y - v2.y);
        return dx * dx + dy * dy;
    }

    public static function distance(v1:Vec2, v2:Vec2) :Float {
        return sqrt(distanceSquared(v1, v2));
    }

    public static var MIN_VALUE (get, null):Float;
	public static var MAX_VALUE (get, null):Float;
	
	private static inline function get_MIN_VALUE ():Float {
		#if flash
		return untyped __global__ ["Number"].MIN_VALUE;
		#elseif js
		return untyped __js__ ("Number.MIN_VALUE");
		#else
		return 2.2250738585072014e-308;
		#end
	}
	
	
	private static inline function get_MAX_VALUE ():Float {
		#if flash
		return untyped __global__ ["Number"].MAX_VALUE;
		#elseif js
		return untyped __js__ ("Number.MAX_VALUE");
		#else
		return 1.7976931348623158e+308;
		#end
	}

}