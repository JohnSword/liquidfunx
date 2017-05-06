package box2d.particle;

import haxe.ds.Vector;

 class StackQueue<T> {

  private var m_buffer : Vector<T>;
  private var m_front : Int = 0;
  private var m_back : Int = 0;
  private var m_end : Int = 0;

  public function new() {}

  public function reset(buffer : Vector<T>) : Void {
    m_buffer = buffer;
    m_front = 0;
    m_back = 0;
    m_end = buffer.length;
  }

  public function push(task : T) : Void {
    if (m_back >= m_end) {
      // TODO: array copy
      Vector.blit(m_buffer, m_front, m_buffer, 0, m_back - m_front);
      // System.arraycopy(m_buffer, m_front, m_buffer, 0, m_back - m_front);
      m_back -= m_front;
      m_front = 0;
      if (m_back >= m_end) {
        return;
      }
    }
    m_buffer[m_back++] = task;
  }

  public function pop() : T {
    return m_buffer[m_front++];
  }

  public function empty() : Bool {
    return m_front >= m_back;
  }

  public function front() : T {
    return m_buffer[m_front];
  }
}

