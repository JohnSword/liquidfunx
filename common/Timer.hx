package box2d.common;

/**
 * Timer for profiling
 * 
 * @author Daniel
 */
class Timer {

    private var resetNanos : Int;

    public function new() {
        reset();
    }

    public function reset() : Void {
        // resetNanos = Sys.time();
        // TODO: resetNanos = System.nanoTime();
    }

    public function getMilliseconds() : Float {
        // return (Sys.time() - resetNanos) / 1000 * 1 / 1000;
        // TODO: return (System.nanoTime() - resetNanos) / 1000 * 1 / 1000;
        return 0;
    }

}