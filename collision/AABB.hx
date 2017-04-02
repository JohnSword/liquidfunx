package box2d.collision;

import box2d.common.MathUtils;
import box2d.common.Settings;
import box2d.common.Vec2;

/** An axis-aligned bounding box. */
class AABB {

     /** Bottom left vertex of bounding box. */
    public var lowerBound : Vec2;
    /** Top right vertex of bounding box. */
    public var upperBound : Vec2;

    /**
   * Creates the default object, with vertices at 0,0 and 0,0.
   */
    public function new(copy:AABB = null) {
        if(copy!=null) {
            // Copies from the given object
            lowerBound = copy.lowerBound.clone();
            upperBound = copy.upperBound.clone();
        } else {
            lowerBound = new Vec2();
            upperBound = new Vec2();
        }
    }

    /**
     * Creates an AABB object using the given bounding vertices.
     * 
     * @param lowerVertex the bottom left vertex of the bounding box
     * @param maxVertex the top right vertex of the bounding box
     */
    public function setVec(lowerVertex : Vec2, upperVertex : Vec2) {
        this.lowerBound = lowerVertex.clone(); // clone to be safe
        this.upperBound = upperVertex.clone();
    }

    /**
     * Sets this object from the given object
     * 
     * @param aabb the object to copy from
     */
    public function set(aabb : AABB) : Void {
        var v : Vec2 = aabb.lowerBound;
        lowerBound.x = v.x;
        lowerBound.y = v.y;
        var v1 : Vec2 = aabb.upperBound;
        upperBound.x = v1.x;
        upperBound.y = v1.y;
    }

    /** Verify that the bounds are sorted */
    public function isValid() : Bool {
        var dx : Float = upperBound.x - lowerBound.x;
        if (dx < 0) {
            return false;
        }
        var dy : Float = upperBound.y - lowerBound.y;
        if (dy < 0) {
            return false;
        }
        return lowerBound.isValid() && upperBound.isValid();
    }

    /**
     * Get the center of the AABB
     * 
     * @return
     */
    public function getCenter() : Vec2 {
        var center : Vec2 = new Vec2(lowerBound.x,lowerBound.y);
        center.addLocalVec(upperBound);
        center.mulLocal(.5);
        return center;
    }

    public function getCenterToOut(final Vec2 out : Vec2) : Void {
        out.x = (lowerBound.x + upperBound.x) * .5;
        out.y = (lowerBound.y + upperBound.y) * .5;
    }

    /**
     * Get the extents of the AABB (half-widths).
     * 
     * @return
     */
     public function getExtents() : Vec2 {
        var center : Vec2 = new Vec2(upperBound.x,upperBound.y);
        center.subLocal(lowerBound);
        center.mulLocal(.5);
        return center;
    }

    public function getExtentsToOut(final Vec2 out) : Void {
        out.x = (upperBound.x - lowerBound.x) * .5f;
        out.y = (upperBound.y - lowerBound.y) * .5f; // thanks FDN1
    }

    public function getVertices(argRay : Array<Vec2>) : Void {
        argRay[0].setVec(lowerBound);
        argRay[1].setVec(lowerBound);
        argRay[1].x += upperBound.x - lowerBound.x;
        argRay[2].setVec(upperBound);
        argRay[3].setVec(upperBound);
        argRay[3].x -= upperBound.x - lowerBound.x;
    }

    /**
     * Combine two AABBs into this one.
     * 
     * @param aabb1
     * @param aab
     */
     public function combine(aabb1 : AABB, aab : AABB) : Void {
        lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x ? aabb1.lowerBound.x : aab.lowerBound.x;
        lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y ? aabb1.lowerBound.y : aab.lowerBound.y;
        upperBound.x = aabb1.upperBound.x > aab.upperBound.x ? aabb1.upperBound.x : aab.upperBound.x;
        upperBound.y = aabb1.upperBound.y > aab.upperBound.y ? aabb1.upperBound.y : aab.upperBound.y;
    }

    /**
     * Gets the perimeter length
     * 
     * @return
     */
     public function getPerimeter() : Float {
        return 2.0 * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
    }

    /**
     * Combines another aabb with this one
     * 
     * @param aabb
     */
     public function combineAABB(aabb : AABB) : Void {
        lowerBound.x = lowerBound.x < aabb.lowerBound.x ? lowerBound.x : aabb.lowerBound.x;
        lowerBound.y = lowerBound.y < aabb.lowerBound.y ? lowerBound.y : aabb.lowerBound.y;
        upperBound.x = upperBound.x > aabb.upperBound.x ? upperBound.x : aabb.upperBound.x;
        upperBound.y = upperBound.y > aabb.upperBound.y ? upperBound.y : aabb.upperBound.y;
    }

    /**
     * Does this aabb contain the provided AABB.
     * 
     * @return
     */
     public function contains(aabb : AABB) : Bool {
        /*
        * boolean result = true; result = result && lowerBound.x <= aabb.lowerBound.x; result = result
        * && lowerBound.y <= aabb.lowerBound.y; result = result && aabb.upperBound.x <= upperBound.x;
        * result = result && aabb.upperBound.y <= upperBound.y; return result;
        */
        // djm: faster putting all of them together, as if one is false we leave the logic
        // early
        return lowerBound.x <= aabb.lowerBound.x && lowerBound.y <= aabb.lowerBound.y
            && aabb.upperBound.x <= upperBound.x && aabb.upperBound.y <= upperBound.y;
    }

    /**
     * @deprecated please use {@link #raycast(RayCastOutput, RayCastInput, IWorldPool)} for better
     *             performance
     * @param output
     * @param input
     * @return
     */
     public function raycast2(output : RayCastOutput, input : RayCastInput) : Bool {
        return raycast(output, input, new DefaultWorldPool(4, 4));
    }

    /**
     * From Real-time Collision Detection, p179.
     * 
     * @param output
     * @param input
     */
     public function raycast(output : RayCastOutput, input : RayCastInput, argPool : IWorldPool) : Bool {
        var tmin : Float = -Float.MAX_VALUE;
        var tmax : Float = Float.MAX_VALUE;

        var p : Vec2 = argPool.popVec2();
        var d : Vec2 = argPool.popVec2();
        var absD : Vec2 = argPool.popVec2();
        var normal : Vec2 = argPool.popVec2();

        p.setVec(input.p1);
        d.setVec(input.p2).subLocal(input.p1);
        Vec2.absToOut(d, absD);

        // x then y
        if (absD.x < Settings.EPSILON) {
            // Parallel.
            if (p.x < lowerBound.x || upperBound.x < p.x) {
                argPool.pushVec2(4);
                return false;
            }
        } else {
            var inv_d : Float = 1.0 / d.x;
            var t1 : Float = (lowerBound.x - p.x) * inv_d;
            var t2 : Float = (upperBound.x - p.x) * inv_d;

            // Sign of the normal vector.
            var s : Float = -1.0;

            if (t1 > t2) {
                var temp : Float = t1;
                t1 = t2;
                t2 = temp;
                s = 1.0;
            }

            // Push the min up
            if (t1 > tmin) {
                normal.setZero();
                normal.x = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = MathUtils.min(tmax, t2);

            if (tmin > tmax) {
                argPool.pushVec2(4);
                return false;
            }
        }

        if (absD.y < Settings.EPSILON) {
            // Parallel.
            if (p.y < lowerBound.y || upperBound.y < p.y) {
                argPool.pushVec2(4);
                return false;
            }
        } else {
            var inv_d : Float = 1.0 / d.y;
            var t1 : Float = (lowerBound.y - p.y) * inv_d;
            var t2 : Float = (upperBound.y - p.y) * inv_d;

            // Sign of the normal vector.
            var s : Float = -1.0;

            if (t1 > t2) {
                var temp : Float = t1;
                t1 = t2;
                t2 = temp;
                s = 1.0;
            }

            // Push the min up
            if (t1 > tmin) {
                normal.setZero();
                normal.y = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = MathUtils.min(tmax, t2);

            if (tmin > tmax) {
                argPool.pushVec2(4);
                return false;
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0.0 || input.maxFraction < tmin) {
            argPool.pushVec2(4);
            return false;
        }

        // Intersection.
        output.fraction = tmin;
        output.normal.x = normal.x;
        output.normal.y = normal.y;
        argPool.pushVec2(4);
        return true;
    }

    public static function testOverlap(a : AABB, b : AABB) : Bool {
        if (b.lowerBound.x - a.upperBound.x > 0.0 || b.lowerBound.y - a.upperBound.y > 0.0) {
            return false;
        }

        if (a.lowerBound.x - b.upperBound.x > 0.0 || a.lowerBound.y - b.upperBound.y > 0.0) {
            return false;
        }

        return true;
    }

    public function toString() : String {
        var s : String = "AABB[" + lowerBound + " . " + upperBound + "]";
        return s;
    }

}