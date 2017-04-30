package box2d.common;

/**
 * This is the viewport transform used from drawing. Use yFlip if you are drawing from the top-left
 * corner.
 * 
 * @author Daniel
 */
interface IViewportTransform {

  /**
   * @return if the transform flips the y axis
   */
  function isYFlip() : Bool;

  /**
   * @param yFlip if we flip the y axis when transforming
   */
  function setYFlip(yFlip:Bool) : Void;

  /**
   * This is the half-width and half-height. This should be the actual half-width and half-height,
   * not anything transformed or scaled. Not a copy.
   */
  function getExtents() : Vec2;

  /**
   * This sets the half-width and half-height. This should be the actual half-width and half-height,
   * not anything transformed or scaled.
   */
  function setExtentsVec(extents:Vec2) : Void;

  /**
   * This sets the half-width and half-height of the viewport. This should be the actual half-width
   * and half-height, not anything transformed or scaled.
   */
  function setExtents(halfWidth:Float, halfHeight:Float) : Void;

  /**
   * center of the viewport. Not a copy.
   */
  function getCenter() : Vec2;

  /**
   * sets the center of the viewport.
   */
  function setCenterVec(pos:Vec2) : Void;

  /**
   * sets the center of the viewport.
   */
  function setCenter(x:Float, y:Float) : Void;

  /**
   * Sets the transform's center to the given x and y coordinates, and using the given scale.
   */
  function setCamera(x:Float, y:Float, scale:Float) : Void;

  /**
   * Transforms the given directional vector by the viewport transform (not positional)
   */
  function getWorldVectorToScreen(world:Vec2, screen:Vec2) : Void;


  /**
   * Transforms the given directional screen vector back to the world direction.
   */
  function getScreenVectorToWorld(screen:Vec2, world:Vec2) : Void;
  
  function getMat22Representation() : Mat22;


  /**
   * takes the world coordinate (world) puts the corresponding screen coordinate in screen. It
   * should be safe to give the same object as both parameters.
   */
  function getWorldToScreen(world:Vec2, screen:Vec2) : Void;


  /**
   * takes the screen coordinates (screen) and puts the corresponding world coordinates in world. It
   * should be safe to give the same object as both parameters.
   */
  function getScreenToWorld(screen:Vec2, world:Vec2) : Void;

  /**
   * Multiplies the viewport transform by the given Mat22
   */
  function mulByTransform(transform:Mat22) : Void;
}