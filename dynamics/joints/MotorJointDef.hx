package box2d.dynamics.joints;

import box2d.common.Vec2;
import box2d.dynamics.Body;

/**
 * Motor joint definition.
 * 
 * @author dmurph
 */
 class MotorJointDef extends JointDef {
  /**
   * Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
   */
  public var linearOffset : Vec2 = new Vec2();

  /**
   * The bodyB angle minus bodyA angle in radians.
   */
  public var angularOffset : Float = 0;

  /**
   * The maximum motor force in N.
   */
  public var maxForce : Float = 0;

  /**
   * The maximum motor torque in N-m.
   */
  public var maxTorque : Float = 0;

  /**
   * Position correction factor in the range [0,1].
   */
  public var correctionFactor : Float = 0;

  public function new() {
    super(JointType.MOTOR);
    angularOffset = 0;
    maxForce = 1;
    maxTorque = 1;
    correctionFactor = 0.3;
  }

  public function initialize(bA : Body, bB : Body) : Void {
    bodyA = bA;
    bodyB = bB;
    var xB : Vec2 = bodyB.getPosition();
    bodyA.getLocalPointToOut(xB, linearOffset);

    var angleA : Float = bodyA.getAngle();
    var angleB : Float = bodyB.getAngle();
    angularOffset = angleB - angleA;
  }
}

