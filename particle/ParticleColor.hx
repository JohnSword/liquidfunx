package box2d.particle;

import box2d.common.Color3f;

/**
 * Small color object for each particle
 * 
 * @author dmurph
 */
 class ParticleColor {

  public var r : Int = 127;
  public var g : Int = 127;
  public var b : Int = 127;
  public var a : Int = 50;

  public function new(color : Color3f = null) {
    if(color!=null) {
      setColor3f(color);
    }
  }

  public function setColor3f(color : Color3f) : Void {
    r = cast (255 * color.x);
    g = cast (255 * color.y);
    b = cast (255 * color.z);
    a = cast 255;
  }
  
  public function setParticleColor(color : ParticleColor) : Void {
    r = color.r;
    g = color.g;
    b = color.b;
    a = color.a;
  }
  
  public function isZero() : Bool {
    return r == 0 && g == 0 && b == 0 && a == 0;
  }

  public function set(r : Int, g : Int, b : Int, a : Int) : Void {
    this.r = r;
    this.g = g;
    this.b = b;
    this.a = a;
  }
}

