package box2d.common;

class Settings {

     /** A "close to zero" float epsilon value for use */
    // public static var EPSILON : Float = 1.1920928955078125E-7;
    // public static var EPSILON : Float = 1.1920928955078125E-7f;
    public static var EPSILON:Float = 0.0000001;

    /** The lowest float value in Flash and JS. */
    public static inline var MIN_VALUE_FLOAT = -1.79769313486231e+308;
    // public static var MIN_VALUE_FLOAT:Float = 0.0000000000000001;

    /** The highest float value in Flash and JS. */
    public static inline var MAX_VALUE_FLOAT = 1.79769313486231e+308;
    // public static var MAX_VALUE_FLOAT:Float = 1.79e+308;

    public static var MAX_VALUE_INT:Int = 2147483647;
    // public static var MAX_VALUE_INT:Int = 0x7FFFFFFF;
    public static var MIN_VALUE_INT:Int = -2147483647;
    
    public static var SQUARE_ROOT_OF_TWO:Float = 1.41421356237;

    /** Pi. */
    public static var PI : Float = Math.PI;

    // JBox2D specific settings
    public static var FAST_ABS : Bool = true;
    public static var FAST_FLOOR : Bool = true;
    public static var FAST_CEIL : Bool = true;
    public static var FAST_ROUND : Bool = true;
    public static var FAST_ATAN2 : Bool = true;
    public static var FAST_POW : Bool = true;
    public static var CONTACT_STACK_INIT_SIZE : Int = 10;
    public static var SINCOS_LUT_ENABLED : Bool = true;

    /**
     * smaller the precision, the larger the table. If a small table is used (eg, precision is .006 or
     * greater), make sure you set the table to lerp it's results. Accuracy chart is in the MathUtils
     * source. Or, run the tests yourself in {@link SinCosTest}.</br> </br> Good lerp precision
     * values:
     * <ul>
     * <li>.0092</li>
     * <li>.008201</li>
     * <li>.005904</li>
     * <li>.005204</li>
     * <li>.004305</li>
     * <li>.002807</li>
     * <li>.001508</li>
     * <li>9.32500E-4</li>
     * <li>7.48000E-4</li>
     * <li>8.47000E-4</li>
     * <li>.0005095</li>
     * <li>.0001098</li>
     * <li>9.50499E-5</li>
     * <li>6.08500E-5</li>
     * <li>3.07000E-5</li>
     * <li>1.53999E-5</li>
     * </ul>
     */
    public static var SINCOS_LUT_PRECISION : Float = .00011;
    public static var SINCOS_LUT_LENGTH : Int = Math.ceil(Math.PI * 2 / SINCOS_LUT_PRECISION);
     
    /**
     * Use if the table's precision is large (eg .006 or greater). Although it is more expensive, it
     * greatly increases accuracy. Look in the MathUtils source for some test results on the accuracy
     * and speed of lerp vs non lerp. Or, run the tests yourself in {@link SinCosTest}.
     */
    public static var SINCOS_LUT_LERP : Bool = false;

    // Collision

    /**
     * The maximum number of contact points between two convex shapes.
     */
    public static var maxManifoldPoints : Int = 2;

    /**
     * The maximum number of vertices on a convex polygon.
     */
    public static var maxPolygonVertices : Int  = 8;

    /**
     * This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
     * without triggering a tree adjustment. This is in meters.
     */
    public static var aabbExtension : Float  = 0.1;

    /**
     * This is used to fatten AABBs in the dynamic tree. This is used to predict the future position
     * based on the current displacement. This is a dimensionless multiplier.
     */
    public static var aabbMultiplier : Float  = 2.0;

    /**
     * A small length used as a collision and constraint tolerance. Usually it is chosen to be
     * numerically significant, but visually insignificant.
     */
    public static var linearSlop : Float = 0.005; // 0.5 cm

    /**
     * A small angle used as a collision and constraint tolerance. Usually it is chosen to be
     * numerically significant, but visually insignificant.
     */
    public static var angularSlop : Float = 2.0 / 180 * Math.PI;

    /**
     * The radius of the polygon/edge shape skin. This should not be modified. Making this smaller
     * means polygons will have and insufficient for continuous collision. Making it larger may create
     * artifacts for vertex collision.
     */
    public static var polygonRadius : Float = 2.0 * 0.005;
    // public static var polygonRadius : Float = (2.0 * linearSlop);

    /** Maximum number of sub-steps per contact in continuous physics simulation. */
    public static var maxSubSteps : Int = 8;

    // Dynamics

    /**
     * Maximum number of contacts to be handled to solve a TOI island.
     */
    public static var maxTOIContacts : Int = 32;

    /**
     * A velocity threshold for elastic collisions. Any collision with a relative linear velocity
     * below this threshold will be treated as inelastic.
     */
    public static var velocityThreshold : Float = 1.0;

    /**
     * The maximum linear position correction used when solving constraints. This helps to prevent
     * overshoot.
     */
    public static var maxLinearCorrection : Float = 0.2;

    /**
     * The maximum angular position correction used when solving constraints. This helps to prevent
     * overshoot.
     */
    public static var maxAngularCorrection : Float = 8.0 / 180.0 * Math.PI;

    /**
     * The maximum linear velocity of a body. This limit is very large and is used to prevent
     * numerical problems. You shouldn't need to adjust this.
     */
    public static var maxTranslation : Float = 2.0;
    public static var maxTranslationSquared : Float = 4.0;
    // public static var maxTranslationSquared : Float = (maxTranslation * maxTranslation);

    /**
     * The maximum angular velocity of a body. This limit is very large and is used to prevent
     * numerical problems. You shouldn't need to adjust this.
     */
    public static var maxRotation : Float = 0.5 * Math.PI;
    public static var maxRotationSquared : Float = (0.5 * Math.PI) * (0.5 * Math.PI);
    // public static var maxRotationSquared : Float = (maxRotation * maxRotation);

    /**
     * This scale factor controls how fast overlap is resolved. Ideally this would be 1 so that
     * overlap is removed in one time step. However using values close to 1 often lead to overshoot.
     */
    public static var baumgarte : Float = 0.2;
    public static var toiBaugarte : Float = 0.75;


    // Sleep

    /**
     * The time that a body must be still before it will go to sleep.
     */
    public static var timeToSleep : Float = 0.5;

    /**
     * A body cannot sleep if its linear velocity is above this tolerance.
     */
    public static var linearSleepTolerance : Float = 0.01;

    /**
     * A body cannot sleep if its angular velocity is above this tolerance.
     */
    public static var angularSleepTolerance : Float = 2.0 / 180.0 * Math.PI;


    // Particle

    /**
     * A symbolic constant that stands for particle allocation error.
     */
    public static var invalidParticleIndex : Int = -1;

    /**
     * The standard distance between particles, divided by the particle radius.
     */
    public static var particleStride : Float = 0.75;

    /**
     * The minimum particle weight that produces pressure.
     */
    public static var minParticleWeight : Float = 1.0;

    /**
     * The upper limit for particle weight used in pressure calculation.
     */
    public static var maxParticleWeight : Float = 5.0;

    /**
     * The maximum distance between particles in a triad, divided by the particle radius.
     */
    public static var maxTriadDistance : Int = 2;
    public static var maxTriadDistanceSquared : Int = 4;
    // public static var maxTriadDistanceSquared : Int = (maxTriadDistance * maxTriadDistance);

    /**
     * The initial size of particle data buffers.
     */
    public static var minParticleBufferCapacity : Int = 256;

    /**
     * Friction mixing law. Feel free to customize this. TODO djm: add customization
     * 
     * @param friction1
     * @param friction2
     * @return
     */
    public static function mixFriction(friction1 : Float, friction2 : Float) : Float {
        return MathUtils.sqrt(friction1 * friction2);
    }

    /**
     * Restitution mixing law. Feel free to customize this. TODO djm: add customization
     * 
     * @param restitution1
     * @param restitution2
     * @return
     */
    public static function mixRestitution(restitution1 : Float, restitution2 : Float) : Float {
        return restitution1 > restitution2 ? restitution1 : restitution2;
    }

    public function new() {

    }

}