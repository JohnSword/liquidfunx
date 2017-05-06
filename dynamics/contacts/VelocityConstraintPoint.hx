package box2d.dynamics.contacts;

import box2d.common.Vec2;

class VelocityConstraintPoint {
    public var rA : Vec2 = new Vec2();
    public var rB : Vec2 = new Vec2();
    public var normalImpulse : Float = 0;
    public var tangentImpulse : Float = 0;
    public var normalMass : Float = 0;
    public var tangentMass : Float = 0;
    public var velocityBias : Float = 0;
    public function new() {}
  }