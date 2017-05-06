package box2d.particle;

import box2d.common.MathUtils;
import box2d.common.Vec2;
import box2d.pooling.normal.MutableStack;

import haxe.ds.Vector;

 class VoronoiDiagram {

  private var m_generatorBuffer : Vector<Generator>;
  private var m_generatorCount : Int = 0;
  private var m_countY : Int = 0;
  private var m_countX : Int = 0;
  // The diagram is an array of "pointers".
  private var m_diagram : Vector<Generator>;

  public function new(generatorCapacity : Int) {
    m_generatorBuffer = new Vector<Generator>(generatorCapacity);
    for(i in 0 ... generatorCapacity) {
      m_generatorBuffer[i] = new Generator();
    }
    m_generatorCount = 0;
    m_countX = 0;
    m_countY = 0;
    m_diagram = null;
  }

  public function getNodes(callback : VoronoiDiagramCallback) : Void {
    var y : Int = 0;
    while (y < m_countY - 1) {
      var x : Int = 0;
      while (x < m_countX - 1) {
        var i : Int = x + y * m_countX;
        var a : Generator = m_diagram[i];
        var b : Generator = m_diagram[i + 1];
        var c : Generator = m_diagram[i + m_countX];
        var d : Generator = m_diagram[i + 1 + m_countX];
        if (b != c) {
          if (a != b && a != c) {
            callback.callback(a.tag, b.tag, c.tag);
          }
          if (d != b && d != c) {
            callback.callback(b.tag, d.tag, c.tag);
          }
        }
        x++;
      }
      y++;
    }
  }

  public function addGenerator(center : Vec2, tag : Int) : Void {
    var g : Generator = m_generatorBuffer[m_generatorCount++];
    g.center.x = center.x;
    g.center.y = center.y;
    g.tag = tag;
  }

  private var lower : Vec2 = new Vec2();
  private var upper : Vec2 = new Vec2();
  private var taskPool : MutableStack<VoronoiDiagramTask> =
      new MutableStack<VoronoiDiagramTask>(50);
      // TODO: how can this work in haxe?
      // {
      //   override public function newInstance() : VoronoiDiagramTask {
      //     return new VoronoiDiagramTask();
      //   }

      //   override public function newArray(size : Int) : Vector<VoronoiDiagramTask> {
      //     return new Vector<VoronoiDiagramTask>(size);
      //   }
      // };
  private var queue : StackQueue<VoronoiDiagramTask> = new StackQueue<VoronoiDiagramTask>();

  public function generate(radius : Float) : Void {
    var inverseRadius : Float = 1 / radius;
    lower.x = MathUtils.MAX_VALUE;
    lower.y = MathUtils.MAX_VALUE;
    upper.x = -MathUtils.MAX_VALUE;
    upper.y = -MathUtils.MAX_VALUE;
    for (k in 0 ... m_generatorCount) {
      var g : Generator = m_generatorBuffer[k];
      Vec2.minToOut(lower, g.center, lower);
      Vec2.maxToOut(upper, g.center, upper);
    }
    m_countX = 1 + cast (inverseRadius * (upper.x - lower.x));
    m_countY = 1 + cast (inverseRadius * (upper.y - lower.y));
    m_diagram = new Vector<Generator>(m_countX * m_countY);
    queue.reset(new Vector<VoronoiDiagramTask>(4 * m_countX * m_countX));
    // queue.reset(new VoronoiDiagramTask[4 * m_countX * m_countX]);
    for (k in 0 ... m_generatorCount) {
      var g : Generator = m_generatorBuffer[k];
      g.center.x = inverseRadius * (g.center.x - lower.x);
      g.center.y = inverseRadius * (g.center.y - lower.y);
      var x : Int = MathUtils.max(0, MathUtils.min(g.center.x, m_countX - 1));
      var y : Int = MathUtils.max(0, MathUtils.min(g.center.y, m_countY - 1));
      queue.push(taskPool.pop().set(x, y, x + y * m_countX, g));
    }
    while (!queue.empty()) {
      var front : VoronoiDiagramTask = queue.pop();
      var x : Int = front.m_x;
      var y : Int = front.m_y;
      var i : Int = front.m_i;
      var g : Generator = front.m_generator;
      if (m_diagram[i] == null) {
        m_diagram[i] = g;
        if (x > 0) {
          queue.push(taskPool.pop().set(x - 1, y, i - 1, g));
        }
        if (y > 0) {
          queue.push(taskPool.pop().set(x, y - 1, i - m_countX, g));
        }
        if (x < m_countX - 1) {
          queue.push(taskPool.pop().set(x + 1, y, i + 1, g));
        }
        if (y < m_countY - 1) {
          queue.push(taskPool.pop().set(x, y + 1, i + m_countX, g));
        }
      }
      taskPool.push(front);
    }
    var maxIteration : Int = m_countX + m_countY;
    for (iteration in 0 ... maxIteration) {
      for (y in 0 ... m_countY) {
        var x : Int = 0;
        while (x < m_countX - 1) {
          var i : Int = x + y * m_countX;
          var a : Generator = m_diagram[i];
          var b : Generator = m_diagram[i + 1];
          if (a != b) {
            queue.push(taskPool.pop().set(x, y, i, b));
            queue.push(taskPool.pop().set(x + 1, y, i + 1, a));
          }
          x++;
        }
      }
      var y : Int = 0;
      while (y < m_countY - 1) {
        for (x in 0 ... m_countX) {
          var i : Int = x + y * m_countX;
          var a : Generator = m_diagram[i];
          var b : Generator = m_diagram[i + m_countX];
          if (a != b) {
            queue.push(taskPool.pop().set(x, y, i, b));
            queue.push(taskPool.pop().set(x, y + 1, i + m_countX, a));
          }
        }
        y++;
      }
      var updated : Bool = false;
      while (!queue.empty()) {
        var front : VoronoiDiagramTask = queue.pop();
        var x : Int = front.m_x;
        var y : Int = front.m_y;
        var i : Int = front.m_i;
        var k : Generator = front.m_generator;
        var a : Generator = m_diagram[i];
        var b : Generator = k;
        if (a != b) {
          var ax : Float = a.center.x - x;
          var ay : Float = a.center.y - y;
          var bx : Float = b.center.x - x;
          var by : Float = b.center.y - y;
          var a2 : Float = ax * ax + ay * ay;
          var b2 : Float = bx * bx + by * by;
          if (a2 > b2) {
            m_diagram[i] = b;
            if (x > 0) {
              queue.push(taskPool.pop().set(x - 1, y, i - 1, b));
            }
            if (y > 0) {
              queue.push(taskPool.pop().set(x, y - 1, i - m_countX, b));
            }
            if (x < m_countX - 1) {
              queue.push(taskPool.pop().set(x + 1, y, i + 1, b));
            }
            if (y < m_countY - 1) {
              queue.push(taskPool.pop().set(x, y + 1, i + m_countX, b));
            }
            updated = true;
          }
        }
        taskPool.push(front);
      }
      if (!updated) {
        break;
      }
    }
  }
}

