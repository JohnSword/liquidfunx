package box2d.common;

class Color3f {

    public static var WHITE : Color3f = new Color3f(1, 1, 1);
	public static var BLACK : Color3f = new Color3f(0, 0, 0);
	public static var BLUE : Color3f= new Color3f(0, 0, 1);
	public static var GREEN : Color3f = new Color3f(0, 1, 0);
	public static var RED : Color3f = new Color3f(1, 0, 0);
	
	public var x : Float;
	public var y : Float;
	public var z : Float;

    public function new(r:Float=0,g:Float=0,b:Float=0) {
        this.x = r;
        this.y = g;
        this.z = b;
    }

    public function set(float r, float g, float b) : Void {
		x = r;
		y = g;
		z = b;
	}
	
	public function setColor(argColor:Color3f) : Void {
		x = argColor.x;
		y = argColor.y;
		z = argColor.z;
	}

}