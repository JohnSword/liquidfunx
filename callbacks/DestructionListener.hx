package box2d.callbacks;

import box2d.dynamics.joints.Joint;

/**
 * Joints and fixtures are destroyed when their associated
 * body is destroyed. Implement this listener so that you
 * may nullify references to these joints and shapes.
 * @author Daniel Murphy
 */

import box2d.dynamics.Fixture;

interface DestructionListener {

    /**
	 * Called when any joint is about to be destroyed due
	 * to the destruction of one of its attached bodies.
	 * @param joint
	 */
	function sayGoodbyeJoint(joint : Joint) : Void;
	
	/**
	 * Called when any fixture is about to be destroyed due
	 * to the destruction of its parent body.
	 * @param fixture
	 */
	function sayGoodbyeFixture(fixture : Fixture) : Void;

}