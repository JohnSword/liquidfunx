package box2d.common;

import haxe.ds.Vector;

class BufferUtils {

    public static function reallocateBuffer<T:Dynamic>(klass : Class<T>, oldBuffer : Vector<T>, oldCapacity : Int, newCapacity : Int) : Vector<T> {
        var newBuffer : Vector<T> = new Vector<T>(newCapacity);
        if (oldBuffer != null) {
            // TODO: array copy
            Vector.blit(oldBuffer, 0, newBuffer, 0, oldCapacity);
            // System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
        }
        for (i in oldCapacity ... newCapacity) {
        try {
            newBuffer[i] = Type.createInstance(klass, []);
        } catch (e : Dynamic) {
            throw e;
        }
        }
        return newBuffer;
    }

}