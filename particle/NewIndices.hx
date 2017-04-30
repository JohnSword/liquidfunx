package box2d.particle;

class NewIndices {
    private var start : Int;
    private var mid : Int;
    private var end : Int;

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