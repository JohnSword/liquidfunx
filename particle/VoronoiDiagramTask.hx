package box2d.particle;

class VoronoiDiagramTask {
    public var m_x : Int = 0;
    public var m_y : Int = 0; 
    public var  m_i : Int = 0;
    public var m_generator : Generator;

    public function new(x : Int = 0, y : Int = 0, i : Int = 0, g : Generator = null) {
      m_x = x;
      m_y = y;
      m_i = i;
      m_generator = g;
    }

    public function set(x : Int, y : Int, i : Int, g : Generator) : VoronoiDiagramTask {
      m_x = x;
      m_y = y;
      m_i = i;
      m_generator = g;
      return this;
    }
  }