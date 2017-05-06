package box2d.collision;

enum ContactType {
    VERTEX; 
    FACE;
}

/**
 * Contact ids to facilitate warm starting. Note: the ContactFeatures class is just embedded in here
 */
class ContactID {

    public var indexA : Int = 0;
    public var indexB : Int = 0;
    public var typeA : Int = 0;
    public var typeB : Int = 0;

    public function new(c : ContactID = null) {
        if(c!=null) {
            set(c);
        }
    }

    public function getKey() : Int {
        return (indexA) << 24 | (indexB) << 16 | (typeA) << 8 | (typeB);
    }

    public function isEqual(cid : ContactID) : Bool {
        return getKey() == cid.getKey();
    }

    public function set(c : ContactID) : Void {
        indexA = c.indexA;
        indexB = c.indexB;
        typeA = c.typeA;
        typeB = c.typeB;
    }

    public function flip() : Void {
        var tempA : Int = indexA;
        indexA = indexB;
        indexB = tempA;
        tempA = typeA;
        typeA = typeB;
        typeB = tempA;
    }

    /**
     * zeros out the data
     */
     public function zero() : Void {
        indexA = 0;
        indexB = 0;
        typeA = 0;
        typeB = 0;
    }

    public function compareTo(o : ContactID) : Int {
        return getKey() - o.getKey();
    }

}