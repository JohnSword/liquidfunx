package box2d.callbacks;

import box2d.dynamics.Fixture;
import box2d.dynamics.Filter;

// updated to rev 100
/**
 * Implement this class to provide collision filtering. In other words, you can implement
 * this class if you want finer control over contact creation.
 * @author Daniel Murphy
 */
class ContactFilter {

    public function new() {

    }

    /**
	 * Return true if contact calculations should be performed between these two shapes.
	 * @warning for performance reasons this is only called when the AABBs begin to overlap.
	 * @param fixtureA
	 * @param fixtureB
	 * @return
	 */
	public function shouldCollide(fixtureA : Fixture, fixtureB : Fixture) : Bool {
		var filterA : Filter = fixtureA.getFilterData();
		var filterB : Filter = fixtureB.getFilterData();

		if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0){
			return filterA.groupIndex > 0;
		}

		var collide : Bool = (filterA.maskBits & filterB.categoryBits) != 0 &&
						  (filterA.categoryBits & filterB.maskBits) != 0;
		return collide;
	}

}