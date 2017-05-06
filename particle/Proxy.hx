package box2d.particle;

/** Used for detecting particle contacts */
class Proxy {
    public var index : Int = 0;
    public var tag : Int = 0;

    public function compareTo(o : Proxy) : Int {
        return (tag - o.tag) < 0 ? -1 : (o.tag == tag ? 0 : 1);
    }

    public function equals(obj : Dynamic) : Bool {
        if (this == obj) return true;
        if (obj == null) return false;
        if (Type.getClass(this) != Type.getClass(obj)) return false;
        var other : Proxy = cast obj;
        if (tag != other.tag) return false;
        return true;
    }
}