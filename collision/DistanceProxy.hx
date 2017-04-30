package box2d.collision;

import box2d.common.Vec2;
import box2d.common.Settings;
import box2d.collision.shapes.Shape;
import box2d.collision.shapes.CircleShape;
import box2d.collision.shapes.PolygonShape;
import box2d.collision.shapes.ChainShape;
import box2d.collision.shapes.EdgeShape;
import haxe.ds.Vector;

/**
 * A distance proxy is used by the GJK algorithm. It encapsulates any shape. TODO: see if we can
 * just do assignments with m_vertices, instead of copying stuff over
 * 
 * @author daniel
 */
class DistanceProxy {

    public var m_vertices : Vector<Vec2>;
    public var m_count : Int;
    public var m_radius : Float;
    public var m_buffer : Vector<Vec2>;

    public function new () {
      m_vertices = new Vector<Vec2>(Settings.maxPolygonVertices);
      for(i in 0 ... m_vertices.length) {
        m_vertices[i] = new Vec2();
      }
      m_buffer = new Vector<Vec2>(2);
      m_count = 0;
      m_radius = 0;
    }

    /**
     * Initialize the proxy using the given shape. The shape must remain in scope while the proxy is
     * in use.
     */
    public function set(shape : Shape, index : Int) : Void {
      switch (shape.getType()) {
      case CIRCLE:
        var circle : CircleShape = cast shape;
        m_vertices[0].setVec(circle.m_p);
        m_count = 1;
        m_radius = circle.m_radius;

      case POLYGON:
        var poly : PolygonShape = cast shape;
        m_count = poly.m_count;
        m_radius = poly.m_radius;
        for(i in 0 ... m_count) {
          m_vertices[i].setVec(poly.m_vertices[i]);
        }
       
      case CHAIN:
        var chain : ChainShape = cast shape;

        m_buffer[0] = chain.m_vertices[index];
        if (index + 1 < chain.m_count) {
          m_buffer[1] = chain.m_vertices[index + 1];
        } else {
          m_buffer[1] = chain.m_vertices[0];
        }

        m_vertices[0].setVec(m_buffer[0]);
        m_vertices[1].setVec(m_buffer[1]);
        m_count = 2;
        m_radius = chain.m_radius;
       
      case EDGE:
        var edge : EdgeShape = cast shape;
        m_vertices[0].setVec(edge.m_vertex1);
        m_vertices[1].setVec(edge.m_vertex2);
        m_count = 2;
        m_radius = edge.m_radius;
       
      default:
      }
    }

    /**
     * Get the supporting vertex index in the given direction.
     * 
     * @param d
     * @return
     */
    public function getSupport(d : Vec2) : Int {
      var bestIndex : Int = 0;
      var bestValue : Float = Vec2.dot(m_vertices[0], d);
      for(i in 1 ... m_count) {
        var value : Float = Vec2.dot(m_vertices[i], d);
        if (value > bestValue) {
          bestIndex = i;
          bestValue = value;
        }
      }

      return bestIndex;
    }

    /**
     * Get the supporting vertex in the given direction.
     * 
     * @param d
     * @return
     */
    public function getSupportVertex(d : Vec2) : Vec2 {
      var bestIndex : Int = 0;
      var bestValue : Float = Vec2.dot(m_vertices[0], d);
      for(i in 1 ... m_count) {
        var value : Float = Vec2.dot(m_vertices[i], d);
        if (value > bestValue) {
          bestIndex = i;
          bestValue = value;
        }
      }

      return m_vertices[bestIndex];
    }

    /**
     * Get the vertex count.
     * 
     * @return
     */
    public function getVertexCount() : Int {
      return m_count;
    }

    /**
     * Get a vertex by index. Used by Distance.
     * 
     * @param index
     * @return
     */
    public function getVertex(index : Int) : Vec2 {
      return m_vertices[index];
    }
  }