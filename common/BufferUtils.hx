package box2d.common;

import box2d.particle.ParticleBodyContact;
import box2d.particle.ParticleContact;
import box2d.particle.Proxy;
import box2d.particle.ParticleGroup;
import haxe.ds.Vector;

class BufferUtils {

    public static function reallocateBuffer(klass : Class<Dynamic>, oldBuffer : Vector<Dynamic>, oldCapacity : Int, newCapacity : Int) : Dynamic {
        var newBuffer : Vector<Dynamic> = new Vector<Dynamic>(newCapacity);
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

    public static function reallocateProxyBuffer(klass : Class<Dynamic>, oldBuffer : Vector<Proxy>, oldCapacity : Int, newCapacity : Int) : Vector<Proxy> {
        var newBuffer : Vector<Proxy> = new Vector<Proxy>(newCapacity);
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

    public static function reallocateParticleGroupBuffer(klass : Class<Dynamic>, oldBuffer : Vector<ParticleGroup>, oldCapacity : Int, newCapacity : Int) : Vector<ParticleGroup> {
        var newBuffer : Vector<ParticleGroup> = new Vector<ParticleGroup>(newCapacity);
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

    public static function reallocateParticleContactBuffer(klass : Class<Dynamic>, oldBuffer : Vector<ParticleContact>, oldCapacity : Int, newCapacity : Int) : Vector<ParticleContact> {
        var newBuffer : Vector<ParticleContact> = new Vector<ParticleContact>(newCapacity);
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

    public static function reallocateParticleBodyContactBuffer(klass : Class<Dynamic>, oldBuffer : Vector<ParticleBodyContact>, oldCapacity : Int, newCapacity : Int) : Vector<ParticleBodyContact> {
        var newBuffer : Vector<ParticleBodyContact> = new Vector<ParticleBodyContact>(newCapacity);
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

    public static function reallocateVec2ContactBuffer(klass : Class<Dynamic>, oldBuffer : Vector<Vec2>, oldCapacity : Int, newCapacity : Int) : Vector<Vec2> {
        var newBuffer : Vector<Vec2> = new Vector<Vec2>(newCapacity);
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

    /** Reallocate a buffer. */
    public static function reallocateBufferInt(oldBuffer : Vector<Int>, oldCapacity : Int, newCapacity : Int) : Vector<Int> {
        var newBuffer : Vector<Int> = new Vector<Int>(newCapacity);
        if (oldBuffer != null) {
            // TODO: array copy
            Vector.blit(oldBuffer, 0, newBuffer, 0, oldCapacity);
            //   System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
        }
        return newBuffer;
    }

     /** Reallocate a buffer. */
    public static function reallocateBufferFloat(oldBuffer : Vector<Float>, oldCapacity : Int, newCapacity : Int) : Vector<Float> {
        var newBuffer : Vector<Float> = new Vector<Float>(newCapacity);
        if (oldBuffer != null) {
            // TODO: array copy
            Vector.blit(oldBuffer, 0, newBuffer, 0, oldCapacity);
            //   System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
        }
        return newBuffer;
    }

    /**
     * Reallocate a buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
     public static function reallocateBufferDeffered<T>(klass : Class<Dynamic>, buffer : Vector<Dynamic>, userSuppliedCapacity : Int,
         oldCapacity : Int, newCapacity : Int, deferred : Bool) : Dynamic {
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(klass, buffer, oldCapacity, newCapacity);
        }
        return buffer;
    }

     public static function reallocateVec2BufferDeffered<T>(klass : Class<Dynamic>, buffer : Vector<Vec2>, userSuppliedCapacity : Int,
         oldCapacity : Int, newCapacity : Int, deferred : Bool) : Vector<Vec2> {
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateVec2ContactBuffer(klass, buffer, oldCapacity, newCapacity);
        }
        return buffer;
    }

    public static function reallocateParticleGroupBufferDeffered(klass : Class<Dynamic>, buffer : Vector<ParticleGroup>, userSuppliedCapacity : Int,
         oldCapacity : Int, newCapacity : Int, deferred : Bool) : Vector<ParticleGroup> {
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateParticleGroupBuffer(klass, buffer, oldCapacity, newCapacity);
        }
        return buffer;
    }

    /**
     * Reallocate an int buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
     public static function reallocateBufferIntDeffered(buffer : Vector<Int>, userSuppliedCapacity : Int, oldCapacity : Int,
         newCapacity : Int, deferred : Bool) : Vector<Int> {
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBufferInt(buffer, oldCapacity, newCapacity);
        }
        return buffer;
    }

    /**
     * Reallocate a float buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
     public static function reallocateBufferFloatDeffered(buffer : Vector<Float>, userSuppliedCapacity : Int, oldCapacity : Int,
         newCapacity : Int, deferred : Bool) : Vector<Float> {
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBufferFloat(buffer, oldCapacity, newCapacity);
        }
        return buffer;
    }

    /** Rotate an array, see std::rotate */
    public static function rotateInt(ray : Vector<Int>, first : Int, new_first : Int, last : Int) : Void {
        var next : Int = new_first;
        while (next != first) {
            var temp : Int = ray[first];
            ray[first] = ray[next];
            ray[next] = temp;
            first++;
            next++;
            if (next == last) {
                next = new_first;
            } else if (first == new_first) {
                new_first = next;
            }
        }
    }

    /** Rotate an array, see std::rotate */
    public static function rotateFloat(ray : Vector<Float>, first : Int, new_first : Int, last : Int) : Void {
        var next : Int = new_first;
        while (next != first) {
            var temp : Float = ray[first];
            ray[first] = ray[next];
            ray[next] = temp;
            first++;
            next++;
            if (next == last) {
                next = new_first;
            } else if (first == new_first) {
                new_first = next;
            }
        }
    }

    public static function rotateDynamic(ray : Vector<Dynamic>, first : Int, new_first : Int, last : Int) : Void {
        var next : Int = new_first;
        while (next != first) {
            var temp : Dynamic = ray[first];
            ray[first] = ray[next];
            ray[next] = temp;
            first++;
            next++;
            if (next == last) {
                next = new_first;
            } else if (first == new_first) {
                new_first = next;
            }
        }
    }

}