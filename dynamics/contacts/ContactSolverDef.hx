package box2d.dynamics.contacts;

import haxe.ds.Vector;

class ContactSolverDef {
    public var step : TimeStep;
    public var contacts : Vector<Contact>;
    public var count : Int = 0;
    public var positions : Vector<Position>;
    public var velocities : Vector<Velocity>;
    public function new() {}
}