package box2d.common;

class Color3f {

    public static var WHITE : Color3f = new Color3f(1, 1, 1);
	public static var BLACK : Color3f = new Color3f(0, 0, 0);
	public static var BLUE : Color3f= new Color3f(0, 0, 1);
	public static var GREEN : Color3f = new Color3f(0, 1, 0);
	public static var RED : Color3f = new Color3f(1, 0, 0);
	
	public var x : Float = 0;
	public var y : Float = 0;
	public var z : Float = 0;

	public var color (get, null):Int;

    public function new(r:Float=0,g:Float=0,b:Float=0) {
        this.x = r;
        this.y = g;
        this.z = b;
    }

    public function set(r : Float, g : Float, b : Float) : Void {
		x = r;
		y = g;
		z = b;
	}
	
	public function setColor(argColor:Color3f) : Void {
		x = argColor.x;
		y = argColor.y;
		z = argColor.z;
	}

	// Color
	private function get_color() : Int{
		var r:Int = Std.int(x*255);
		var g:Int = Std.int(y*255);
		var b:Int = Std.int(z*255);
		return (r << 16) | (g << 8) | (b);
	}

}