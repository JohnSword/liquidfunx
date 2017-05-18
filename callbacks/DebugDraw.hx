package box2d.callbacks;

import box2d.common.IViewportTransform;
import box2d.common.Vec2;
import box2d.common.Color3f;
import box2d.common.Transform;
import box2d.particle.ParticleColor;

import haxe.ds.Vector;

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


    private var m_drawFlags : Int  = 0;
    private var viewportTransform : IViewportTransform;

    public function new(viewport:IViewportTransform = null) {
        this.m_drawFlags = 0;
        this.viewportTransform = viewport;
    }

    public function setViewportTransform(viewportTransform : IViewportTransform) : Void {
        this.viewportTransform = viewportTransform;
    }

    public function setFlags(flags  : Int) : Void {
        m_drawFlags = flags;
    }

    public function getFlags() : Int {
        return m_drawFlags;
    }

    public function appendFlags(flags  : Int) : Void {
        m_drawFlags |= flags;
    }

    public function clearFlags(flags  : Int) : Void {
        m_drawFlags &= ~flags;
    }

    private var temp : Vec2 = new Vec2();

    /**
   * Draw a closed polygon provided in CCW order. This implementation uses
   * {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the polygon.
   * 
   * @param vertices
   * @param vertexCount
   * @param color
   */
  public function drawPolygon(vertices : Vector<Vec2>, vertexCount : Int, color : Color3f) : Void {
    if (vertexCount == 1) {
      // getWorldToScreenToOut(vertices[i], temp);
      drawSegment(vertices[0], vertices[0], color);
      return;
    }

    for(i in 0 ... (vertexCount - 1)) {
      // getWorldToScreenToOut(vertices[i], temp);
      drawSegment(vertices[i], vertices[i + 1], color);
    }

    if (vertexCount > 2) {
      drawSegment(vertices[vertexCount - 1], vertices[0], color);
    }
  }

  public function drawPoint(argPoint : Vec2, argRadiusOnScreen : Float, argColor : Color3f) : Void {
    
  }

  /**
   * Draw a solid closed polygon provided in CCW order.
   * 
   * @param vertices
   * @param vertexCount
   * @param color
   */
  public function drawSolidPolygon(vertices : Vector<Vec2>, vertexCount : Int, color : Color3f) : Void {
    
  }

  /**
   * Draw a circle.
   * 
   * @param center
   * @param radius
   * @param color
   */
  public function drawCircle(center : Vec2, radius : Float, color : Color3f) : Void {
    
  }

  /** Draws a circle with an axis */
  public function drawCircle2(center : Vec2, radius : Float, axis : Vec2, color : Color3f) : Void {
    drawCircle(center, radius, color);
  }

  /**
   * Draw a solid circle.
   * 
   * @param center
   * @param radius
   * @param axis
   * @param color
   */
  public function drawSolidCircle(center : Vec2, radius : Float, axis : Vec2, color : Color3f) : Void {
    
  }

  /**
   * Draw a line segment.
   * 
   * @param p1
   * @param p2
   * @param color
   */
  public function drawSegment(p1 : Vec2, p2 : Vec2, color : Color3f) : Void {
    
  }

  /**
   * Draw a transform. Choose your own length scale
   * 
   * @param xf
   */
  public function drawTransform(xf : Transform) : Void {
    
  }

  /**
   * Draw a string.
   * 
   * @param x
   * @param y
   * @param s
   * @param color
   */
  public function drawString(x : Float, y : Float, s : String, color : Color3f) : Void {
    
  }

  /**
   * Draw a particle array
   * 
   * @param colors can be null
   */
  public function drawParticles(centers : Vector<Vec2>, radius : Float, colors : Vector<ParticleColor>, count : Int) : Void {
    
  }

  /**
   * Draw a particle array
   * 
   * @param colors can be null
   */
  public function drawParticlesWireframe(centers : Vector<Vec2>, radius : Float, colors : Vector<ParticleColor>, count : Int) : Void {
    
  }

  public function clear() : Void {}

  /** Called at the end of drawing a world */
  public function flush() : Void {}

  public function drawString2(pos : Vec2, s : String, color : Color3f) : Void {
    drawString(pos.x, pos.y, s, color);
  }

  public function getViewportTranform() : IViewportTransform {
    return viewportTransform;
  }

  /**
   * @param x
   * @param y
   * @param scale
   * @deprecated use the viewport transform in {@link #getViewportTranform()}
   */
  public function setCamera(x : Float, y : Float, scale : Float) : Void {
    viewportTransform.setCamera(x, y, scale);
  }


  /**
   * @param argScreen
   * @param argWorld
   */
  public function getScreenToWorldToOut(argScreen : Vec2, argWorld : Vec2) : Void {
    viewportTransform.getScreenToWorld(argScreen, argWorld);
  }

  /**
   * @param argWorld
   * @param argScreen
   */
  public function getWorldToScreenToOut(argWorld : Vec2, argScreen : Vec2) : Void {
    viewportTransform.getWorldToScreen(argWorld, argScreen);
  }

  /**
   * Takes the world coordinates and puts the corresponding screen coordinates in argScreen.
   * 
   * @param worldX
   * @param worldY
   * @param argScreen
   */
  public function getWorldToScreenToOut2(worldX : Float, worldY : Float, argScreen : Vec2) : Void {
    argScreen.set(worldX, worldY);
    viewportTransform.getWorldToScreen(argScreen, argScreen);
  }

  /**
   * takes the world coordinate (argWorld) and returns the screen coordinates.
   * 
   * @param argWorld
   */
  public function getWorldToScreen(argWorld : Vec2) : Vec2 {
    var screen : Vec2 = new Vec2();
    viewportTransform.getWorldToScreen(argWorld, screen);
    return screen;
  }

  /**
   * Takes the world coordinates and returns the screen coordinates.
   * 
   * @param worldX
   * @param worldY
   */
  public function getWorldToScreen2(worldX : Float, worldY : Float) : Vec2 {
    var argScreen : Vec2 = new Vec2(worldX, worldY);
    viewportTransform.getWorldToScreen(argScreen, argScreen);
    return argScreen;
  }

  /**
   * takes the screen coordinates and puts the corresponding world coordinates in argWorld.
   * 
   * @param screenX
   * @param screenY
   * @param argWorld
   */
  public function getScreenToWorldToOut2(screenX : Float, screenY : Float, argWorld : Vec2) : Void {
    argWorld.set(screenX, screenY);
    viewportTransform.getScreenToWorld(argWorld, argWorld);
  }

  /**
   * takes the screen coordinates (argScreen) and returns the world coordinates
   * 
   * @param argScreen
   */
  public function getScreenToWorld(argScreen : Vec2) : Vec2 {
    var world : Vec2 = new Vec2();
    viewportTransform.getScreenToWorld(argScreen, world);
    return world;
  }

  /**
   * takes the screen coordinates and returns the world coordinates.
   * 
   * @param screenX
   * @param screenY
   */
  public function getScreenToWorld2(screenX : Float, screenY : Float) : Vec2 {
    var screen : Vec2 = new Vec2(screenX, screenY);
    viewportTransform.getScreenToWorld(screen, screen);
    return screen;
  }

}