package box2d.common;

/**
 * Timer for profiling
 * 
 * @author Daniel
 */
class TimerB2d {

    private var resetNanos : Float = 0;

    public function new() {
        reset();
    }

    public function reset() : Void {
        // TODO: resetNanos = System.nanoTime();
        #if cpp
        resetNanos = Sys.time();
        #else
        resetNanos = Date.now().getTime();
        #end 
    }

    public function getMilliseconds() : Float {
        // TODO: return (System.nanoTime() - resetNanos) / 1000 * 1 / 1000;
        #if cpp
        // return (Sys.time() - resetNanos) / 1000 * 1 / 1000;
        return Sys.time() * 1000.0;
        #else
        return Date.now().getTime();
        // return (Date.now().getTime() - resetNanos) / 1000 * 1 / 1000;
        #end 
    }

}