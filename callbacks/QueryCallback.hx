package box2d.callbacks;

import box2d.dynamics.Fixture;

/**
 * Callback class for AABB queries.
 * See {@link World#queryAABB(QueryCallback, org.jbox2d.collision.AABB)}.
 * @author Daniel Murphy
 */
interface QueryCallback {

	/**
	 * Called for each fixture found in the query AABB.
	 * @param fixture
	 * @return false to terminate the query.
	 */
	function reportFixture(fixture : Fixture) : Bool;
}