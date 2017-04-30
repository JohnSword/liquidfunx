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
import box2d.dynamics.contacts.Contact;

interface IWorldPool {

	function getPolyContactStack() : IDynamicStack<Contact>;

	function getCircleContactStack() : IDynamicStack<Contact>;

	function getPolyCircleContactStack() : IDynamicStack<Contact>;
	
    function getEdgeCircleContactStack() : IDynamicStack<Contact>;
    
    function getEdgePolyContactStack() : IDynamicStack<Contact>;

    function getChainCircleContactStack() : IDynamicStack<Contact>;
    
    function getChainPolyContactStack() : IDynamicStack<Contact>;
    
	function popVec2() : Vec2;

	function popVec2s(num : Int) : Array<Vec2>;

	function pushVec2(num : Int) : Void;

	function popVec3() : Vec3;

	function popVec3s(num : Int) : Array<Vec3> ;

	function pushVec3(num : Int) : Void;

	function popMat22() : Mat22;

	function popMat22s(num : Int) : Array<Mat22>;

	function pushMat22(num : Int) : Void;
	
	function popMat33() : Mat33;
	
	function pushMat33(num : Int) : Void;

	function popAABB() : AABB;

	function popAABBs(num : Int) : Array<AABB>;

	function pushAABB(num : Int) : Void;
	
	function popRot() : Rot;

	function pushRot(num : Int) : Void;
	
	function getCollision() : Collision;

	function getTimeOfImpact() : TimeOfImpact;

	function getDistance() : Distance;

	function getFloatArray(argLength : Int) : Array<Float>;

	function getIntArray(argLength : Int) : Array<Int>;

	function getVec2Array(argLength : Int) : Array<Vec2>;
}