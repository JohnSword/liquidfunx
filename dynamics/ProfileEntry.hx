package box2d.dynamics;

import box2d.common.MathUtils;
import box2d.dynamics.Profile;

class ProfileEntry {
    public var longAvg : Float;
    public var shortAvg : Float;
    public var min : Float;
    public var max : Float;
    public var accum : Float;

    public function new() {
      min = MathUtils.MAX_VALUE;
      max = -MathUtils.MAX_VALUE;
    }

    public function record(value : Float) : Void {
      longAvg = longAvg * (1 - Profile.LONG_FRACTION) + value * Profile.LONG_FRACTION;
      shortAvg = shortAvg * (1 - Profile.SHORT_FRACTION) + value * Profile.SHORT_FRACTION;
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