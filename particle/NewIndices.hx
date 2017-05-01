package box2d.particle;

class NewIndices {
    public var start : Int;
    public var mid : Int;
    public var end : Int;

    public function new() {}

    public function getIndex(i: Int) : Int {
      if (i < start) {
        return i;
      } else if (i < mid) {
        return i + end - mid;
      } else if (i < end) {
        return i + start - mid;
      } else {
        return i;
      }
    }
  }