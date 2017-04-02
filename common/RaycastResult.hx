package box2d.common;

class RaycastResult {

    public var lambda : Float = 0.0;
	public var normal : Vec2 = new Vec2();
	
	public function set(argOther : RaycastResult) : RaycastResult {
		lambda = argOther.lambda;
		normal.setVec( argOther.normal);
		return this;
	}

    public function new() {
    }

}