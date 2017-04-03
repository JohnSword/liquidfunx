package box2d.callbacks;

/**
 * Implement this abstract class to allow JBox2d to automatically draw your physics for debugging
 * purposes. Not intended to replace your own custom rendering routines!
 * 
 * @author Daniel Murphy
 */
class DebugDraw {

    /** Draw shapes */
    public static var e_shapeBit : Int = 1 << 1;
    /** Draw joint connections */
    public static var e_jointBit : Int = 1 << 2;
    /** Draw axis aligned bounding boxes */
    public static var e_aabbBit : Int = 1 << 3;
    /** Draw pairs of connected objects */
    public static var e_pairBit : Int = 1 << 4;
    /** Draw center of mass frame */
    public static var e_centerOfMassBit : Int = 1 << 5;
    /** Draw dynamic tree */
    public static var e_dynamicTreeBit : Int = 1 << 6;
    /** Draw only the wireframe for drawing performance */
    public static var e_wireframeDrawingBit : Int = 1 << 7;


    private m_drawFlags : Int ;
    private viewportTransform : IViewportTransform;

    public function new() {

    }

}