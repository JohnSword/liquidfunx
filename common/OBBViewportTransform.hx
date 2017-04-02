package box2d.common;

class OBB {
    public var R : Mat22 = new Mat22();
    public var center : Vec2 = new Vec2();
    public var extents : Vec2 = new Vec2();
    public function new() {}
}

class OBBViewportTransform implements IViewportTransform {

    public var box : OBB = new OBB();
    private var yFlip : Bool = false;
    private var inv2 : Mat22 = new Mat22();
    private var yFlipMat : Mat22;

    public function new() {
        box.R.setIdentity();
        yFlipMat = new Mat22()
        yFlipMat.fromFloats(1, 0, 0, -1);
    }

    public function set(vpt : OBBViewportTransform) : Void {
        box.center.setVec(vpt.box.center);
        box.extents.setVec(vpt.box.extents);
        box.R.setMat22(vpt.box.R);
        yFlip = vpt.yFlip;
    }

    public function setCamera(x : Float, y : Float, scale : Float) : Void {
        box.center.set(x, y);
        Mat22.createScaleTransformFM(scale, box.R);
    }

    public function getExtents() : Vec2 {
        return box.extents;
    }
    
    public function getMat22Representation() : Mat22 {
        return box.R;
    }

    public function setExtentsVec(argExtents : Vec2) : Void {
        box.extents.setVec(argExtents);
    }

    public function setExtents(halfWidth : Float, halfHeight : Float) : Void {
        box.extents.set(halfWidth, halfHeight);
    }

    public function getCenter() : Vec2 {
        return box.center;
    }

    public function setCenterVec(argPos : Vec2) : Void {
        box.center.setVec(argPos);
    }

    public function setCenter(x : Float, y : Float) : Void {
        box.center.set(x, y);
    }

    /**
     * Gets the transform of the viewport, transforms around the center. Not a copy.
     */
    public function getTransform() : Mat22 {
        return box.R;
    }

    /**
     * Sets the transform of the viewport. Transforms about the center.
     */
    public function setTransform(transform : Mat22) : Void {
        box.R.setMat22(transform);
    }

    /**
     * Multiplies the obb transform by the given transform
     */
    public function mulByTransform(transform : Mat22) : Void {
        box.R.mulLocal(transform);
    }

    public function isYFlip() : Bool {
        return yFlip;
    }

    public function setYFlip(yFlip : Bool) : Void {
        this.yFlip = yFlip;
    }

    private var inv : Mat22 = new Mat22();

    public function getScreenVectorToWorld(screen : Vec2, world : Vec2) : Void {
        box.R.invertToOut(inv);
        inv.mulToOutVec(screen, world);
        if (yFlip) {
            yFlipMat.mulToOutVec(world, world);
        }
    }

    public function getWorldVectorToScreen(world : Vec2, screen : Vec2) : Void {
        box.R.mulToOutVec(world, screen);
        if (yFlip) {
            yFlipMat.mulToOutVec(screen, screen);
        }
    }

    public function getWorldToScreen(world : Vec2, screen : Vec2) : Void {
        screen.x = world.x - box.center.x;
        screen.y = world.y - box.center.y;
        box.R.mulToOutVec(screen, screen);
        if (yFlip) {
            yFlipMat.mulToOutVec(screen, screen);
        }
        screen.x += box.extents.x;
        screen.y += box.extents.y;
    }

    public function getScreenToWorld(screen : Vec2, world : Vec2) : Void {
        world.x = screen.x - box.extents.x;
        world.y = screen.y - box.extents.y;
        if (yFlip) {
            yFlipMat.mulToOutVec(world, world);
        }
        box.R.invertToOut(inv2);
        inv2.mulToOutVec(world, world);
        world.x += box.center.x;
        world.y += box.center.y;
    }

}