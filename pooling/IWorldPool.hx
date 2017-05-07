package box2d.pooling;

import box2d.common.Vec2;
import box2d.common.Vec3;
import box2d.common.Mat22;
import box2d.common.Mat33;
import box2d.common.Rot;
import box2d.collision.AABB;
import box2d.collision.Collision;
import box2d.collision.TimeOfImpact;
import box2d.collision.Distance;
// import box2d.dynamics.contacts.Contact;

import haxe.ds.Vector;

interface IWorldPool {

	function getPolyContactStack() : IDynamicStack;
	// function getPolyContactStack() : IDynamicStack<Contact>;

	function getCircleContactStack() : IDynamicStack;
	// function getCircleContactStack() : IDynamicStack<Contact>;

	function getPolyCircleContactStack() : IDynamicStack;
	// function getPolyCircleContactStack() : IDynamicStack<Contact>;
	
    function getEdgeCircleContactStack() : IDynamicStack;
    // function getEdgeCircleContactStack() : IDynamicStack<Contact>;
    
    function getEdgePolyContactStack() : IDynamicStack;
    // function getEdgePolyContactStack() : IDynamicStack<Contact>;

    function getChainCircleContactStack() : IDynamicStack;
    // function getChainCircleContactStack() : IDynamicStack<Contact>;
    
    function getChainPolyContactStack() : IDynamicStack;
    // function getChainPolyContactStack() : IDynamicStack<Contact>;
    
	function popVec2() : Vec2;

	function popVec2s(num : Int) : Vector<Vec2>;

	function pushVec2(num : Int) : Void;

	function popVec3() : Vec3;

	function popVec3s(num : Int) : Vector<Vec3>;

	function pushVec3(num : Int) : Void;

	function popMat22() : Mat22;

	function popMat22s(num : Int) : Vector<Mat22>;

	function pushMat22(num : Int) : Void;
	
	function popMat33() : Mat33;
	
	function pushMat33(num : Int) : Void;

	function popAABB() : AABB;

	function popAABBs(num : Int) : Vector<AABB>;

	function pushAABB(num : Int) : Void;
	
	function popRot() : Rot;

	function pushRot(num : Int) : Void;
	
	function getCollision() : Collision;

	function getTimeOfImpact() : TimeOfImpact;

	function getDistance() : Distance;

	function getFloatArray(argLength : Int) : Vector<Float>;

	function getIntArray(argLength : Int) : Vector<Int>;

	function getVec2Array(argLength : Int) : Vector<Vec2>;
}