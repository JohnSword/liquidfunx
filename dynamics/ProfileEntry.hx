package box2d.dynamics;

import box2d.common.MathUtils;

class ProfileEntry {

  private static var LONG_AVG_NUMS : Int = 20;
  private static var LONG_FRACTION : Float = 1 / LONG_AVG_NUMS;
  private static var SHORT_AVG_NUMS : Int = 5;
  private static var SHORT_FRACTION = 1 / SHORT_AVG_NUMS;

    public var longAvg : Float = 0;
    public var shortAvg : Float = 0;
    public var min : Float = 0;
    public var max : Float = 0;
    public var accum : Float = 0;

    public function new() {
      min = MathUtils.MAX_VALUE;
      max = -MathUtils.MAX_VALUE;
    }

    public function record(value : Float) : Void {
      longAvg = longAvg * (1 - LONG_FRACTION) + value * LONG_FRACTION;
      shortAvg = shortAvg * (1 - SHORT_FRACTION) + value * SHORT_FRACTION;
      min = MathUtils.min(value, min);
      max = MathUtils.max(value, max);
    }

    public function startAccum() : Void {
      accum = 0;
    }

    public function accumValue(value : Float) : Void {
      accum += value;
    }

    public function endAccum() : Void {
      record(accum);
    }

    public function toString() : String {
        // TODO: format string
      return "%.2f (%.2f) [%.2f,%.2f]";
    //   return String.format("%.2f (%.2f) [%.2f,%.2f]", shortAvg, longAvg, min, max);
    }
  }