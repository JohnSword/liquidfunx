package box2d.common;

/**
 * <p>Provides utility routines to simplify obtaining randomized values.</p>
 */
class Random {

    private var seed : Float;
    private var M : Float = 2147483647;
    private var A : Float = 16807;
    private var Q : Float;
    private var R : Float;
    private var low : Float;
    private var high : Float;

    /**
     * Construct a Randoms.
     */
    public function new () {
        Q = ( M / A );
        R = ( M % A );
        seed = 1;
		low = 0;
		high = 10;
    }

    public function next() : Int {
        seed = A * ( seed % Q ) - R * ( seed / Q );
        if ( seed <= 0 ) {
            seed += M;
        }
        return Std.int(seed);
    }


    public function nextInt( n : Int ) : Int {
        high = n;
        return Std.int( low + ( next() % ( high - low ) ) );
    }


    public function nextFloat() : Float {
        var rnd : Float = next() * ( 1.0 / M );
        return rnd;
    }

    /**
     * Returns a pseudorandom, uniformly distributed <code>int</code> value between <code>0</code>
     * (inclusive) and <code>high</code> (exclusive).
     *
     * @param high the high value limiting the random number sought.
     *
     * @throws IllegalArgumentException if <code>high</code> is not positive.
     */
    public function getInt (high : Int) : Int
    {
        return nextInt(high);
    }

    /**
     * Returns a pseudorandom, uniformly distributed <code>int</code> value between
     * <code>low</code> (inclusive) and <code>high</code> (exclusive).
     *
     * @throws IllegalArgumentException if <code>high - low</code> is not positive.
     */
    public function getInRange (low : Int, high : Int) : Int {
        return low + nextInt(high - low);
    }

    /**
     * Returns a pseudorandom, uniformly distributed <code>Number</code> value between
     * <code>0.0</code> (inclusive) and the <code>high</code> (exclusive).
     *
     * @param high the high value limiting the random number sought.
     */
    public function getFloat (high : Float) : Float {
        return nextFloat() * high;
    }

    /**
     * Returns a pseudorandom, uniformly distributed <code>Number</code> value between
     * <code>low</code> (inclusive) and <code>high</code> (exclusive).
     */
    public function getNumberInRange (low : Float, high : Float) : Float {
        return low + (nextFloat() * (high - low));
    }

    /**
     * Returns true approximately one in <code>n</code> times.
     *
     * @throws IllegalArgumentException if <code>n</code> is not positive.
     */
    public function getChance (n : Int) : Bool {
        return (0 == nextInt(n));
    }

    /**
     * Has a probability <code>p</code> of returning true.
     */
    public function getProbability (p : Float) : Bool {
        return nextFloat() < p;
    }

    /**
     * Returns <code>true</code> or <code>false</code> with approximately even probability.
     */
    public function getBoolean () : Bool {
        return getChance(2);
    }

    /**
     * Shuffle the specified array
     */
    public function shuffle (arr : Array<Dynamic>) : Void {
        // starting from the end of the list, repeatedly swap the element in question with a
        // random element previous to it up to and including itself
        var ii : Int = arr.length;
        while(ii > 1) {
            var idx1 : Int = ii - 1;
            var idx2 : Int = getInt(ii);
            var tmp : Dynamic = arr[idx1];
            arr[idx1] = arr[idx2];
            arr[idx2] = tmp;
            ii--;
        }
        // for (var ii : Int = arr.length; ii > 1; ii--) {
        // }
    }

    /**
     * Pick a random element from the specified Array, or return <code>ifEmpty</code>
     * if it is empty.
     */
    public function pick (arr :Array<Dynamic>, ifEmpty : Dynamic = null) : Dynamic {
        if (arr == null || arr.length == 0) {
            return ifEmpty;
        }
        return arr[nextInt(arr.length)];
    }

    /**
     * Pick a random element from the specified Array and remove it from the list, or return
     * <code>ifEmpty</code> if it is empty.
     */
    public function pluck (arr : Array<Dynamic>, ifEmpty : Dynamic = null) : Dynamic {
        if (arr == null || arr.length == 0) {
            return ifEmpty;
        }
        return arr.splice(nextInt(arr.length), 1)[0];
    }

}