package box2d.common;

class PlatformMathUtils {

    private static var SHIFT23 : Float = 1 << 23;
    private static var INV_SHIFT23 : Float = 1.0 / SHIFT23;

    public static function fastPow(a: Float , b: Float ) : Float {
        // TODO: fast pow function
        return Math.pow(a,b);
        // var x : Float = Float.floatToRawIntBits(a);
        // x *= INV_SHIFT23;
        // x -= 127;
        // var y : Float = x - (x >= 0 ? Std.int(x) : Std.int(x) - 1);
        // b *= x + (y - y * y) * 0.346607;
        // y = b - (b >= 0 ? (int) b : (int) b - 1);
        // y = (y - y * y) * 0.33971f;
        // return Float.intBitsToFloat((int) ((b + 127 - y) * SHIFT23));
    }

}