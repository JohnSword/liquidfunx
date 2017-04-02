package box2d.collision;

import box2d.common.Vec2;
import box2d.common.Transform;
import box2d.pooling.IWorldPool;


class Collision {

    public static var NULL_FEATURE : Int = Int.MAX_VALUE;

    private var pool : IWorldPool;

    private var input : DistanceInput = new DistanceInput();
    private var cache : SimplexCache = new SimplexCache();
    private var output : DistanceOutput = new DistanceOutput();

    public function new(argPool : IWorldPool) {
        incidentEdge[0] = new ClipVertex();
        incidentEdge[1] = new ClipVertex();
        clipPoints1[0] = new ClipVertex();
        clipPoints1[1] = new ClipVertex();
        clipPoints2[0] = new ClipVertex();
        clipPoints2[1] = new ClipVertex();
        pool = argPool;
    }

    /**
     * Determine if two generic shapes overlap.
     * 
     * @param shapeA
     * @param shapeB
     * @param xfA
     * @param xfB
     * @return
     */
     public function testOverlap(shapeA : Shape, indexA : Int, shapeB : Shape, indexB : Int, xfA : Transform, xfB : Transform) : Bool {
        input.proxyA.set(shapeA, indexA);
        input.proxyB.set(shapeB, indexB);
        input.transformA.set(xfA);
        input.transformB.set(xfB);
        input.useRadii = true;

        cache.count = 0;

        pool.getDistance().distance(output, cache, input);
        // djm note: anything significant about 10.0f?
        return output.distance < 10.0f * Settings.EPSILON;
    }

    /**
     * Compute the point states given two manifolds. The states pertain to the transition from
     * manifold1 to manifold2. So state1 is either persist or remove while state2 is either add or
     * persist.
     * 
     * @param state1
     * @param state2
     * @param manifold1
     * @param manifold2
     */
     public static function getPointStates(state1 : Array<PointState>, state2 : Array<PointState>, manifold1 : Manifold, manifold2 : Manifold) : Void {

        for (i in 0...Settings.maxManifoldPoints) {
            state1[i] = PointState.NULL_STATE;
            state2[i] = PointState.NULL_STATE;
        }

        // Detect persists and removes.
        for (i in 0...manifold1.pointCount) {
            ContactID id = manifold1.points[i].id;

            state1[i] = PointState.REMOVE_STATE;

            for (j in 0...manifold2.pointCount) {
                if (manifold2.points[j].id.isEqual(id)) {
                    state1[i] = PointState.PERSIST_STATE;
                    break;
                }
            }
        }

        // Detect persists and adds
        for (i in 0...manifold2.pointCount) {
            ContactID id = manifold2.points[i].id;

            state2[i] = PointState.ADD_STATE;

            for (j in 0...manifold1.pointCount) {
                if (manifold1.points[j].id.isEqual(id)) {
                    state2[i] = PointState.PERSIST_STATE;
                    break;
                }
            }
        }
    }

    /**
     * Clipping for contact manifolds. Sutherland-Hodgman clipping.
     * 
     * @param vOut
     * @param vIn
     * @param normal
     * @param offset
     * @return
     */
     public static function clipSegmentToLine(vOut : Array<ClipVertex>, vIn : Array<ClipVertex>, normal : Vec2, offset : Float, vertexIndexA : Int) : Int {
        // Start with no output points
        var numOut : Int = 0;
        var vIn0 : ClipVertex = vIn[0];
        var vIn1 : ClipVertex = vIn[1];
        var vIn0v : Vec2 = vIn0.v;
        var vIn1v : Vec2 = vIn1.v;

        // Calculate the distance of end points to the line
        var distance0 : Float = Vec2.dot(normal, vIn0v) - offset;
        var distance1 : Float = Vec2.dot(normal, vIn1v) - offset;

        // If the points are behind the plane
        if (distance0 <= 0.0) {
            vOut[numOut++].set(vIn0);
        }
        if (distance1 <= 0.0) {
            vOut[numOut++].set(vIn1);
        }

        // If the points are on different sides of the plane
        if (distance0 * distance1 < 0.0) {
            // Find intersection point of edge and plane
            var interp : Float = distance0 / (distance0 - distance1);

            var vOutNO : ClipVertex = vOut[numOut];
            // vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
            vOutNO.v.x = vIn0v.x + interp * (vIn1v.x - vIn0v.x);
            vOutNO.v.y = vIn0v.y + interp * (vIn1v.y - vIn0v.y);

            // VertexA is hitting edgeB.
            vOutNO.id.indexA = vertexIndexA;
            vOutNO.id.indexB = vIn0.id.indexB;
            vOutNO.id.typeA = ContactID.Type.VERTEX.ordinal();
            vOutNO.id.typeB = ContactID.Type.FACE.ordinal();
            ++numOut;
        }

        return numOut;
    }

    // #### COLLISION STUFF (not from collision.h or collision.cpp) ####

    // djm pooling
    private static var d : Vec2 = new Vec2();

    /**
     * Compute the collision manifold between two circles.
     * 
     * @param manifold
     * @param circle1
     * @param xfA
     * @param circle2
     * @param xfB
     */
     public function collideCircles(manifold : Manifold, circle1 : CircleShape,
          xfA : Transform, circle2 : CircleShape, xfB : Transform) : Void {
        manifold.pointCount = 0;

        // after inline:
        var circle1p : Vec2 = circle1.m_p;
        var circle2p : Vec2 = circle2.m_p;
        var pAx : Float = (xfA.q.c * circle1p.x - xfA.q.s * circle1p.y) + xfA.p.x;
        var pAy : Float = (xfA.q.s * circle1p.x + xfA.q.c * circle1p.y) + xfA.p.y;
        var pBx : Float = (xfB.q.c * circle2p.x - xfB.q.s * circle2p.y) + xfB.p.x;
        var pBy : Float = (xfB.q.s * circle2p.x + xfB.q.c * circle2p.y) + xfB.p.y;
        var dx : Float = pBx - pAx;
        var dy : Float = pBy - pAy;
        var distSqr : Float = dx * dx + dy * dy;
        // end inline

        var radius : Float = circle1.m_radius + circle2.m_radius;
        if (distSqr > radius * radius) {
            return;
        }

        manifold.type = ManifoldType.CIRCLES;
        manifold.localPoint.set(circle1p);
        manifold.localNormal.setZero();
        manifold.pointCount = 1;

        manifold.points[0].localPoint.set(circle2p);
        manifold.points[0].id.zero();
    }

    // djm pooling, and from above

    /**
     * Compute the collision manifold between a polygon and a circle.
     * 
     * @param manifold
     * @param polygon
     * @param xfA
     * @param circle
     * @param xfB
     */
     public function collidePolygonAndCircle(manifold : Manifold, polygon : PolygonShape,
        xfA : Transform, circle : CircleShape, xfB : Transform) : Void {
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        // after inline:
        var circlep : Vec2 = circle.m_p;
        var xfBq : Rot = xfB.q;
        var xfAq : Rot = xfA.q;
        var cx : Float = (xfBq.c * circlep.x - xfBq.s * circlep.y) + xfB.p.x;
        var cy : Float = (xfBq.s * circlep.x + xfBq.c * circlep.y) + xfB.p.y;
        var px : Float = cx - xfA.p.x;
        var py : Float = cy - xfA.p.y;
        var cLocalx : Float = (xfAq.c * px + xfAq.s * py);
        var cLocaly : Float = (-xfAq.s * px + xfAq.c * py);
        // end inline

        // Find the min separating edge.
        var normalIndex : Int = 0;
        var separation : Float = -Float.MAX_VALUE;
        var radius : Float = polygon.m_radius + circle.m_radius;
        var vertexCount : Int = polygon.m_count;
        var s : Float;
        var vertices : Array<Vec2> = polygon.m_vertices;
        var normals : Array<Vec2> = polygon.m_normals;

        for(i in 0...vertexCount) {
            // after inline
            var vertex : Vec2 = vertices[i];
            var tempx : Float = cLocalx - vertex.x;
            var tempy : Float = cLocaly - vertex.y;
            s = normals[i].x * tempx + normals[i].y * tempy;

            if (s > radius) {
                // early out
                return;
            }

            if (s > separation) {
                separation = s;
                normalIndex = i;
            }
        }

        // Vertices that subtend the incident face.
        var vertIndex1 : Int = normalIndex;
        var vertIndex2 : Int = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        var v1 : Vec2 = vertices[vertIndex1];
        var v2 : Vec2 = vertices[vertIndex2];

        // If the center is inside the polygon ...
        if (separation < Settings.EPSILON) {
            manifold.pointCount = 1;
            manifold.type = ManifoldType.FACE_A;

            // after inline:
            var normal : Vec2 = normals[normalIndex];
            manifold.localNormal.x = normal.x;
            manifold.localNormal.y = normal.y;
            manifold.localPoint.x = (v1.x + v2.x) * .5f;
            manifold.localPoint.y = (v1.y + v2.y) * .5f;
            var mpoint : ManifoldPoint = manifold.points[0];
            mpoint.localPoint.x = circlep.x;
            mpoint.localPoint.y = circlep.y;
            mpoint.id.zero();
            // end inline

            return;
        }

        // Compute barycentric coordinates
        // after inline:
        var tempX : Float = cLocalx - v1.x;
        var tempY : Float = cLocaly - v1.y;
        var temp2X : Float = v2.x - v1.x;
        var temp2Y : Float = v2.y - v1.y;
        var u1 : Float = tempX * temp2X + tempY * temp2Y;

        var temp3X : Float = cLocalx - v2.x;
        var temp3Y : Float = cLocaly - v2.y;
        var temp4X : Float = v1.x - v2.x;
        var temp4Y : Float = v1.y - v2.y;
        var u2 : Float = temp3X * temp4X + temp3Y * temp4Y;
        // end inline

        if (u1 <= 0) {
            // inlined
            var dx : Float = cLocalx - v1.x;
            var dy : Float = cLocaly - v1.y;
            if (dx * dx + dy * dy > radius * radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.type = ManifoldType.FACE_A;
            // before inline:
            // manifold.localNormal.set(cLocal).subLocal(v1);
            // after inline:
            manifold.localNormal.x = cLocalx - v1.x;
            manifold.localNormal.y = cLocaly - v1.y;
            // end inline
            manifold.localNormal.normalize();
            manifold.localPoint.set(v1);
            manifold.points[0].localPoint.set(circlep);
            manifold.points[0].id.zero();
        } else if (u2 <= 0.0) {
            // inlined
            var dx : Float = cLocalx - v2.x;
            var dy : Float = cLocaly - v2.y;
            if (dx * dx + dy * dy > radius * radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.type = ManifoldType.FACE_A;
            // after inline:
            manifold.localNormal.x = cLocalx - v2.x;
            manifold.localNormal.y = cLocaly - v2.y;
            // end inline
            manifold.localNormal.normalize();
            manifold.localPoint.set(v2);
            manifold.points[0].localPoint.set(circlep);
            manifold.points[0].id.zero();
            } else {
            // after inline:
            var fcx : Float = (v1.x + v2.x) * .5f;
            var fcy : Float = (v1.y + v2.y) * .5f;

            var float tx : Float = cLocalx - fcx;
            var float ty : Float = cLocaly - fcy;
            var Vec2 normal = normals[vertIndex1];
            separation = tx * normal.x + ty * normal.y;
            if (separation > radius) {
                return;
            }
            // end inline

            manifold.pointCount = 1;
            manifold.type = ManifoldType.FACE_A;
            manifold.localNormal.set(normals[vertIndex1]);
            manifold.localPoint.x = fcx; // (faceCenter)
            manifold.localPoint.y = fcy;
            manifold.points[0].localPoint.set(circlep);
            manifold.points[0].id.zero();
        }
    }

    // djm pooling, and from above
    private var temp : Vec2 = new Vec2();
    private var xf : Transform = new Transform();
    private var n : Vec2 = new Vec2();
    private var v1 : Vec2 = new Vec2();

    /**
     * Find the max separation between poly1 and poly2 using edge normals from poly1.
     * 
     * @param edgeIndex
     * @param poly1
     * @param xf1
     * @param poly2
     * @param xf2
     * @return
     */
     public function findMaxSeparation(results : EdgeResults, poly1 : PolygonShape,
        xf1 : Transform, poly2 : PolygonShape, xf2 : Transform) : Void {
        var count1 : Int = poly1.m_count;
        var count2 : Int = poly2.m_count;
        var n1s : Array<Vec2> = poly1.m_normals;
        var v1s : Array<Vec2> = poly1.m_vertices;
        var v2s : Array<Vec2> = poly2.m_vertices;
        
        Transform.mulTransToOutUnsafe(xf2, xf1, xf);
        var xfq : Rot = xf.q;

        var bestIndex : Int = 0;
        var maxSeparation : Float = -Float.MAX_VALUE;
        for(i in 0...count1) {
            // Get poly1 normal in frame2.
            Rot.mulToOutUnsafe(xfq, n1s[i], n);
            Transform.mulToOutUnsafe(xf, v1s[i], v1);

            // Find deepest point for normal i.
            var si : Float = Float.MAX_VALUE;
            for(j in 0...count2) {
                var v2sj : Vec2 = v2s[j];
                var sij : Float = n.x * (v2sj.x - v1.x) + n.y * (v2sj.y - v1.y);
                if (sij < si) {
                    si = sij;
                }
            }
            
            if (si > maxSeparation) {
                maxSeparation = si;
                bestIndex = i;
            }
        }

        results.edgeIndex = bestIndex;
        results.separation = maxSeparation;
    }

    public function findIncidentEdge(c : Array<ClipVertex>, poly1 : PolygonShape,
        xf1 : Transform , edge1 : Int, poly2 : PolygonShape, xf2 : Transform) : Void {
        var count1 : Int = poly1.m_count;
        var normals1 : Array<Vec2> = poly1.m_normals;

        var count2 : Int = poly2.m_count;
        var vertices2 : Array<Vec2> = poly2.m_vertices;
        var normals2 : Array<Vec2> = poly2.m_normals;

        // assert (0 <= edge1 && edge1 < count1);

        var c0 : ClipVertex = c[0];
        var c1 : ClipVertex = c[1];
        var xf1q : Rot = xf1.q;
        var xf2q : Rot = xf2.q;

        // Get the normal of the reference edge in poly2's frame.
        // after inline:
        var v : Vec2 = normals1[edge1];
        var tempx : Float = xf1q.c * v.x - xf1q.s * v.y;
        var tempy : Float = xf1q.s * v.x + xf1q.c * v.y;
        var normal1x : Float = xf2q.c * tempx + xf2q.s * tempy;
        var normal1y : Float = -xf2q.s * tempx + xf2q.c * tempy;

        // end inline

        // Find the incident edge on poly2.
        var index : Int = 0;
        var minDot : Float = Float.MAX_VALUE;
        for(i in 0...count2) {
            var b : Vec2 = normals2[i];
            var dot : Float = normal1x * b.x + normal1y * b.y;
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }

        // Build the clip vertices for the incident edge.
        var i1 : Int = index;
        var i2 : Int = i1 + 1 < count2 ? i1 + 1 : 0;

        var v1 : Vec2 = vertices2[i1];
        var out : Vec2 = c0.v;
        out.x = (xf2q.c * v1.x - xf2q.s * v1.y) + xf2.p.x;
        out.y = (xf2q.s * v1.x + xf2q.c * v1.y) + xf2.p.y;
        c0.id.indexA = edge1;
        c0.id.indexB = i1;
        c0.id.typeA = ContactID.Type.FACE.ordinal();
        c0.id.typeB = ContactID.Type.VERTEX.ordinal();

        var v2 : Vec2 = vertices2[i2];
        var out1 : Vec2 = c1.v;
        out1.x = (xf2q.c * v2.x - xf2q.s * v2.y) + xf2.p.x;
        out1.y = (xf2q.s * v2.x + xf2q.c * v2.y) + xf2.p.y;
        c1.id.indexA = edge1;
        c1.id.indexB = i2;
        c1.id.typeA = ContactID.Type.FACE.ordinal();
        c1.id.typeB = ContactID.Type.VERTEX.ordinal();
    }

    private var results1 : EdgeResults = new EdgeResults();
    private var results2 : EdgeResults = new EdgeResults();
    private var incidentEdge : Array<ClipVertex> = new Array<ClipVertex>();
    private var localTangent : Vec2 = new Vec2();
    private var localNormal : Vec2 = new Vec2();
    private var planePoint : Vec2 = new Vec2();
    private var tangent : Vec2 = new Vec2();
    private var v11 : Vec2 = new Vec2();
    private var v12 : Vec2 = new Vec2();
    private var clipPoints1 : Array<ClipVertex> = new ClipVertex();
    private var clipPoints2 : Array<ClipVertex> = new ClipVertex();

    /**
     * Compute the collision manifold between two polygons.
     * 
     * @param manifold
     * @param polygon1
     * @param xf1
     * @param polygon2
     * @param xf2
     */
     public function collidePolygons(manifold : Manifold, polyA : PolygonShape,
        xfA : Transform, polyB : PolygonShape, xfB : Transform) : Void {
        // Find edge normal of max separation on A - return if separating axis is found
        // Find edge normal of max separation on B - return if separation axis is found
        // Choose reference edge as min(minA, minB)
        // Find incident edge
        // Clip

        // The normal points from 1 to 2

        manifold.pointCount = 0;
        var totalRadius : Float = polyA.m_radius + polyB.m_radius;

        findMaxSeparation(results1, polyA, xfA, polyB, xfB);
        if (results1.separation > totalRadius) {
            return;
        }

        findMaxSeparation(results2, polyB, xfB, polyA, xfA);
        if (results2.separation > totalRadius) {
            return;
        }

        var poly1 : PolygonShape;  // reference polygon
        var poly2 : PolygonShape;  // incident polygon
        var xf1 : Transform, xf2 : Transform;
        var edge1 : Int;                 // reference edge
        var flip : Bool;
        var k_tol : Float = 0.1 * Settings.linearSlop;

        if (results2.separation > results1.separation + k_tol) {
            poly1 = polyB;
            poly2 = polyA;
            xf1 = xfB;
            xf2 = xfA;
            edge1 = results2.edgeIndex;
            manifold.type = ManifoldType.FACE_B;
            flip = true;
        } else {
            poly1 = polyA;
            poly2 = polyB;
            xf1 = xfA;
            xf2 = xfB;
            edge1 = results1.edgeIndex;
            manifold.type = ManifoldType.FACE_A;
            flip = false;
        }
        var xf1q : Rot = xf1.q;

        findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

        var count1 : Int = poly1.m_count;
        var vertices1 : Array<Vec2> = poly1.m_vertices;

        var iv1 : Int = edge1;
        var iv2 : Int = edge1 + 1 < count1 ? edge1 + 1 : 0;
        v11.set(vertices1[iv1]);
        v12.set(vertices1[iv2]);
        localTangent.x = v12.x - v11.x;
        localTangent.y = v12.y - v11.y;
        localTangent.normalize();

        // Vec2 localNormal = Vec2.cross(dv, 1.0f);
        localNormal.x = 1f * localTangent.y;
        localNormal.y = -1f * localTangent.x;

        // Vec2 planePoint = 0.5f * (v11+ v12);
        planePoint.x = (v11.x + v12.x) * .5f;
        planePoint.y = (v11.y + v12.y) * .5f;

        // Rot.mulToOutUnsafe(xf1.q, localTangent, tangent);
        tangent.x = xf1q.c * localTangent.x - xf1q.s * localTangent.y;
        tangent.y = xf1q.s * localTangent.x + xf1q.c * localTangent.y;

        // Vec2.crossToOutUnsafe(tangent, 1f, normal);
        var normalx : Float = 1 * tangent.y;
        var normaly : Float = -1 * tangent.x;


        Transform.mulToOut(xf1, v11, v11);
        Transform.mulToOut(xf1, v12, v12);
        // v11 = Mul(xf1, v11);
        // v12 = Mul(xf1, v12);

        // Face offset
        // float frontOffset = Vec2.dot(normal, v11);
        var frontOffset : Float = normalx * v11.x + normaly * v11.y;

        // Side offsets, extended by polytope skin thickness.
        // float sideOffset1 = -Vec2.dot(tangent, v11) + totalRadius;
        // float sideOffset2 = Vec2.dot(tangent, v12) + totalRadius;
        var sideOffset1 : Float = -(tangent.x * v11.x + tangent.y * v11.y) + totalRadius;
        var sideOffset2 : Float = tangent.x * v12.x + tangent.y * v12.y + totalRadius;

        // Clip incident edge against extruded edge1 side edges.
        // ClipVertex clipPoints1[2];
        // ClipVertex clipPoints2[2];
        var np : Int;

        // Clip to box side 1
        // np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);
        tangent.negateLocal();
        np = clipSegmentToLine(clipPoints1, incidentEdge, tangent, sideOffset1, iv1);
        tangent.negateLocal();

        if (np < 2) {
            return;
        }

        // Clip to negative box side 1
        np = clipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

        if (np < 2) {
            return;
        }

        // Now clipPoints2 contains the clipped points.
        manifold.localNormal.set(localNormal);
        manifold.localPoint.set(planePoint);

        var pointCount : Int = 0;
        for(i in 0...Settings.maxManifoldPoints) {
            // for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
            // float separation = Vec2.dot(normal, clipPoints2[i].v) - frontOffset;
            var separation : Float = normalx * clipPoints2[i].v.x + normaly * clipPoints2[i].v.y - frontOffset;

            if (separation <= totalRadius) {
                var cp : ManifoldPoint = manifold.points[pointCount];
                // cp.m_localPoint = MulT(xf2, clipPoints2[i].v);
                var out : Vec2 = cp.localPoint;
                var px : Float = clipPoints2[i].v.x - xf2.p.x;
                var py : Float = clipPoints2[i].v.y - xf2.p.y;
                out.x = (xf2.q.c * px + xf2.q.s * py);
                out.y = (-xf2.q.s * px + xf2.q.c * py);
                cp.id.set(clipPoints2[i].id);
                if (flip) {
                    // Swap features
                    cp.id.flip();
                }
                ++pointCount;
            }
        }

        manifold.pointCount = pointCount;
    }

    private var Q : Vec2 = new Vec2();
    private var e : Vec2 = new Vec2();
    private var cf : ContactID = new ContactID();
    private var e1 : Vec2 = new Vec2();
    private var P : Vec2 = new Vec2();

    // Compute contact points for edge versus circle.
    // This accounts for edge connectivity.
    public function collideEdgeAndCircle(manifold : Manifold, edgeA : EdgeShape, xfA : Transform,
        circleB : CircleShape, xfB : Transform) : Void {
        manifold.pointCount = 0;

        // Compute circle in frame of edge
        // Vec2 Q = MulT(xfA, Mul(xfB, circleB.m_p));
        Transform.mulToOutUnsafe(xfB, circleB.m_p, temp);
        Transform.mulTransToOutUnsafe(xfA, temp, Q);

        var A : Vec2 = edgeA.m_vertex1;
        var B : Vec2 = edgeA.m_vertex2;
        e.set(B).subLocal(A);

        // Barycentric coordinates
        var u : Float = Vec2.dot(e, temp.set(B).subLocal(Q));
        var v : Float = Vec2.dot(e, temp.set(Q).subLocal(A));

        var radius : Float = edgeA.m_radius + circleB.m_radius;

        // ContactFeature cf;
        cf.indexB = 0;
        cf.typeB = (byte) ContactID.Type.VERTEX.ordinal();

        // Region A
        if (v <= 0.0) {
            var P : Vec2 = A;
            d.set(Q).subLocal(P);
            var dd : Float = Vec2.dot(d, d);
            if (dd > radius * radius) {
                return;
            }

            // Is there an edge connected to A?
            if (edgeA.m_hasVertex0) {
                var A1 : Vec2 = edgeA.m_vertex0;
                var B1 : Vec2 = A;
                e1.set(B1).subLocal(A1);
                var u1 : Float = Vec2.dot(e1, temp.set(B1).subLocal(Q));

                // Is the circle in Region AB of the previous edge?
                if (u1 > 0.0) {
                    return;
                }
            }

            cf.indexA = 0;
            cf.typeA = ContactID.Type.VERTEX.ordinal();
            manifold.pointCount = 1;
            manifold.type = Manifold.ManifoldType.CIRCLES;
            manifold.localNormal.setZero();
            manifold.localPoint.set(P);
            // manifold.points[0].id.key = 0;
            manifold.points[0].id.set(cf);
            manifold.points[0].localPoint.set(circleB.m_p);
            return;
        }

        // Region B
        if (u <= 0.0) {
            var P : Vec2 = B;
            d.set(Q).subLocal(P);
            var dd : Float = Vec2.dot(d, d);
            if (dd > radius * radius) {
                return;
            }

            // Is there an edge connected to B?
            if (edgeA.m_hasVertex3) {
                var B2 : Vec2 = edgeA.m_vertex3;
                var A2 : Vec2 = B;
                var e2 : Vec2 = e1;
                e2.set(B2).subLocal(A2);
                var v2 : Float = Vec2.dot(e2, temp.set(Q).subLocal(A2));

                // Is the circle in Region AB of the next edge?
                if (v2 > 0.0) {
                    return;
                }
            }

            cf.indexA = 1;
            cf.typeA = (byte) ContactID.Type.VERTEX.ordinal();
            manifold.pointCount = 1;
            manifold.type = Manifold.ManifoldType.CIRCLES;
            manifold.localNormal.setZero();
            manifold.localPoint.set(P);
            // manifold.points[0].id.key = 0;
            manifold.points[0].id.set(cf);
            manifold.points[0].localPoint.set(circleB.m_p);
            return;
        }

        // Region AB
        var den : Float = Vec2.dot(e, e);
        // assert (den > 0.0f);

        // Vec2 P = (1.0f / den) * (u * A + v * B);
        P.set(A).mulLocal(u).addLocal(temp.set(B).mulLocal(v));
        P.mulLocal(1.0 / den);
        d.set(Q).subLocal(P);
        var dd : Float = Vec2.dot(d, d);
        if (dd > radius * radius) {
            return;
        }

        n.x = -e.y;
        n.y = e.x;
        if (Vec2.dot(n, temp.set(Q).subLocal(A)) < 0.0) {
            n.set(-n.x, -n.y);
        }
        n.normalize();

        cf.indexA = 0;
        cf.typeA = ContactID.Type.FACE.ordinal();
        manifold.pointCount = 1;
        manifold.type = Manifold.ManifoldType.FACE_A;
        manifold.localNormal.set(n);
        manifold.localPoint.set(A);
        // manifold.points[0].id.key = 0;
        manifold.points[0].id.set(cf);
        manifold.points[0].localPoint.set(circleB.m_p);
    }

    private var collider : EPCollider = new EPCollider();

    public function collideEdgeAndPolygon(manifold : Manifold, edgeA : EdgeShape, xfA : Transform,
        polygonB : PolygonShape, xfB : Transform) : Void {
        collider.collide(manifold, edgeA, xfA, polygonB, xfB);
    }

}


 /**
 * This is used for determining the state of contact points.
 * 
 * @author Daniel Murphy
 */
 enum PointState {
    /**
     * point does not exist
     */
    NULL_STATE;
    /**
     * point was added in the update
     */
    ADD_STATE;
    /**
     * point persisted across the update
     */
    PERSIST_STATE;
    /**
     * point was removed in the update
     */
    REMOVE_STATE;
}

enum Type {
    UNKNOWN;
    EDGE_A;
    EDGE_B;
}

 /**
 * Java-specific class for returning edge results
 */
class EdgeResults {
    public var separation : Float;
    public var edgeIndex : Int;
    public function new() {}
}

/**
 * Used for computing contact manifolds.
 */
class ClipVertex {
    public var v : Vec2;
    public var id : ContactID;

    public function new() {
        v = new Vec2();
        id = new ContactID();
    }

    public function set(cv : ClipVertex) {
        var v1 : Vec2 = cv.v;
        v.x = v1.x;
        v.y = v1.y;
        var c : ContactID = cv.id;
        id.indexA = c.indexA;
        id.indexB = c.indexB;
        id.typeA = c.typeA;
        id.typeB = c.typeB;
    }
}

/**
* This structure is used to keep track of the best separating axis.
*/
typedef EPAxis = { 
    type : Type, 
    index : Int,
    separation : Float
}

/**
 * This holds polygon B expressed in frame A.
 */
 class TempPolygon {
    var vertices : Array<Vec2> = new Array<Vec2>(Settings.maxPolygonVertices);
    var normals : Array<Vec2> = new Array<Vec2>(Settings.maxPolygonVertices);
    var count : Int;

    public function new() {
        for(i in 0...vertices.length) {
            vertices[i] = new Vec2();
            normals[i] = new Vec2();
        }
    }
}

 /**
 * Reference face used for clipping
 */
 class ReferenceFace {
    var i1 : Int, i2 : Int;
    var v1 : Vec2 = new Vec2();
    var v2 : Vec2 = new Vec2();
    var normal : Vec2 = new Vec2();

    var sideNormal1 : : Vec2 = new Vec2();
    var sideOffset1 : Float;

    var sideNormal2 : Vec2 = new Vec2();
    var sideOffset2 : Float;

    public function new() {}
}

enum VertexType {
    ISOLATED; 
    CONCAVE; 
    CONVEX;
}

/**
 * This class collides and edge and a polygon, taking into account edge adjacency.
 */
class EPCollider {

    public var m_polygonB : TempPolygon = new TempPolygon();

    var m_xf : Transform = new Transform();
    var m_centroidB : Vec2 = new Vec2();
    var m_v0 : Vec2 = new Vec2();
    var m_v1 : Vec2 = new Vec2();
    var m_v2 : Vec2 = new Vec2();
    var m_v3 : Vec2 = new Vec2();
    var m_normal0 : Vec2 = new Vec2();
    var m_normal1 : Vec2 = new Vec2();
    var m_normal2 : Vec2 = new Vec2();
    var m_normal : Vec2 = new Vec2();

    var m_type1 : VertexType, m_type2 : VertexType;

    var m_lowerLimit : Vec2 = new Vec2();
    var m_upperLimit : Vec2 = new Vec2();
    var m_radius : Float;
    var m_front : Bool;

    public EPCollider() {
        for(i in 0..2) {
        // for (int i = 0; i < 2; i++) {
            ie[i] = new ClipVertex();
            clipPoints1[i] = new ClipVertex();
            clipPoints2[i] = new ClipVertex();
        }
    }

    private var edge1 : Vec2 = new Vec2();
    private var temp : Vec2 = new Vec2();
    private var edge0 : Vec2 = new Vec2();
    private var edge2 : Vec2 = new Vec2();
    private var ie : Array<ClipVertex> = new Array<ClipVertex>(2);
    private var clipPoints1 : Array<ClipVertex> = new Array<ClipVertex>(2);
    private var clipPoints2 : Array<ClipVertex> = new Array<ClipVertex>(2);
    private var rf : ReferenceFace = new ReferenceFace();
    private var edgeAxis : EPAxis = new EPAxis();
    private var polygonAxis : EPAxis = new EPAxis();

    public function collide(manifold : Manifold, edgeA : EdgeShape, xfA : Transform,
        polygonB : PolygonShape, xfB : Transform) {

        Transform.mulTransToOutUnsafe(xfA, xfB, m_xf);
        Transform.mulToOutUnsafe(m_xf, polygonB.m_centroid, m_centroidB);

        m_v0 = edgeA.m_vertex0;
        m_v1 = edgeA.m_vertex1;
        m_v2 = edgeA.m_vertex2;
        m_v3 = edgeA.m_vertex3;

        var hasVertex0 : Bool = edgeA.m_hasVertex0;
        var hasVertex3 : Bool = edgeA.m_hasVertex3;

        edge1.set(m_v2).subLocal(m_v1);
        edge1.normalize();
        m_normal1.set(edge1.y, -edge1.x);
        var offset1 : Float = Vec2.dot(m_normal1, temp.set(m_centroidB).subLocal(m_v1));
        var offset0 : Float = 0.0, offset2 : Float = 0.0;
        var convex1 : Bool = false, convex2 : Bool = false;

        // Is there a preceding edge?
        if (hasVertex0) {
            edge0.set(m_v1).subLocal(m_v0);
            edge0.normalize();
            m_normal0.set(edge0.y, -edge0.x);
            convex1 = Vec2.cross(edge0, edge1) >= 0.0f;
            offset0 = Vec2.dot(m_normal0, temp.set(m_centroidB).subLocal(m_v0));
        }

        // Is there a following edge?
        if (hasVertex3) {
            edge2.set(m_v3).subLocal(m_v2);
            edge2.normalize();
            m_normal2.set(edge2.y, -edge2.x);
            convex2 = Vec2.cross(edge1, edge2) > 0.0;
            offset2 = Vec2.dot(m_normal2, temp.set(m_centroidB).subLocal(m_v2));
        }

        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3) {
            if (convex1 && convex2) {
                m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal0.x;
                    m_lowerLimit.y = m_normal0.y;
                    m_upperLimit.x = m_normal2.x;
                    m_upperLimit.y = m_normal2.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal1.x;
                    m_lowerLimit.y = -m_normal1.y;
                    m_upperLimit.x = -m_normal1.x;
                    m_upperLimit.y = -m_normal1.y;
                }
            } else if (convex1) {
                m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal0.x;
                    m_lowerLimit.y = m_normal0.y;
                    m_upperLimit.x = m_normal1.x;
                    m_upperLimit.y = m_normal1.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal2.x;
                    m_lowerLimit.y = -m_normal2.y;
                    m_upperLimit.x = -m_normal1.x;
                    m_upperLimit.y = -m_normal1.y;
                }
            } else if (convex2) {
                m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal1.x;
                    m_lowerLimit.y = m_normal1.y;
                    m_upperLimit.x = m_normal2.x;
                    m_upperLimit.y = m_normal2.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal1.x;
                    m_lowerLimit.y = -m_normal1.y;
                    m_upperLimit.x = -m_normal0.x;
                    m_upperLimit.y = -m_normal0.y;
                }
            } else {
                m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal1.x;
                    m_lowerLimit.y = m_normal1.y;
                    m_upperLimit.x = m_normal1.x;
                    m_upperLimit.y = m_normal1.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal2.x;
                    m_lowerLimit.y = -m_normal2.y;
                    m_upperLimit.x = -m_normal0.x;
                    m_upperLimit.y = -m_normal0.y;
                }
            }
        } else if (hasVertex0) {
            if (convex1) {
                m_front = offset0 >= 0.0 || offset1 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal0.x;
                    m_lowerLimit.y = m_normal0.y;
                    m_upperLimit.x = -m_normal1.x;
                    m_upperLimit.y = -m_normal1.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = m_normal1.x;
                    m_lowerLimit.y = m_normal1.y;
                    m_upperLimit.x = -m_normal1.x;
                    m_upperLimit.y = -m_normal1.y;
                }
            } else {
                m_front = offset0 >= 0.0 && offset1 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = m_normal1.x;
                    m_lowerLimit.y = m_normal1.y;
                    m_upperLimit.x = -m_normal1.x;
                    m_upperLimit.y = -m_normal1.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = m_normal1.x;
                    m_lowerLimit.y = m_normal1.y;
                    m_upperLimit.x = -m_normal0.x;
                    m_upperLimit.y = -m_normal0.y;
                }
            }
        } else if (hasVertex3) {
            if (convex2) {
                m_front = offset1 >= 0.0 || offset2 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = -m_normal1.x;
                    m_lowerLimit.y = -m_normal1.y;
                    m_upperLimit.x = m_normal2.x;
                    m_upperLimit.y = m_normal2.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal1.x;
                    m_lowerLimit.y = -m_normal1.y;
                    m_upperLimit.x = m_normal1.x;
                    m_upperLimit.y = m_normal1.y;
                }
            } else {
                m_front = offset1 >= 0.0 && offset2 >= 0.0;
                if (m_front) {
                    m_normal.x = m_normal1.x;
                    m_normal.y = m_normal1.y;
                    m_lowerLimit.x = -m_normal1.x;
                    m_lowerLimit.y = -m_normal1.y;
                    m_upperLimit.x = m_normal1.x;
                    m_upperLimit.y = m_normal1.y;
                } else {
                    m_normal.x = -m_normal1.x;
                    m_normal.y = -m_normal1.y;
                    m_lowerLimit.x = -m_normal2.x;
                    m_lowerLimit.y = -m_normal2.y;
                    m_upperLimit.x = m_normal1.x;
                    m_upperLimit.y = m_normal1.y;
                }
            }
        } else {
            m_front = offset1 >= 0.0;
            if (m_front) {
                m_normal.x = m_normal1.x;
                m_normal.y = m_normal1.y;
                m_lowerLimit.x = -m_normal1.x;
                m_lowerLimit.y = -m_normal1.y;
                m_upperLimit.x = -m_normal1.x;
                m_upperLimit.y = -m_normal1.y;
            } else {
                m_normal.x = -m_normal1.x;
                m_normal.y = -m_normal1.y;
                m_lowerLimit.x = m_normal1.x;
                m_lowerLimit.y = m_normal1.y;
                m_upperLimit.x = m_normal1.x;
                m_upperLimit.y = m_normal1.y;
            }
        }

        // Get polygonB in frameA
        m_polygonB.count = polygonB.m_count;
        for(i in 0...polygonB.m_count) {
        // for (int i = 0; i < polygonB.m_count; ++i) {
            Transform.mulToOutUnsafe(m_xf, polygonB.m_vertices[i], m_polygonB.vertices[i]);
            Rot.mulToOutUnsafe(m_xf.q, polygonB.m_normals[i], m_polygonB.normals[i]);
        }

        m_radius = 2.0 * Settings.polygonRadius;

        manifold.pointCount = 0;

        computeEdgeSeparation(edgeAxis);

        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type == EPAxis.Type.UNKNOWN) {
            return;
        }

        if (edgeAxis.separation > m_radius) {
            return;
        }

        computePolygonSeparation(polygonAxis);
        if (polygonAxis.type != EPAxis.Type.UNKNOWN && polygonAxis.separation > m_radius) {
            return;
        }

        // Use hysteresis for jitter reduction.
        var k_relativeTol : Float = 0.98;
        var k_absoluteTol : Float = 0.001;

        var primaryAxis : EPAxis;
        if (polygonAxis.type == EPAxis.Type.UNKNOWN) {
            primaryAxis = edgeAxis;
        } else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol) {
            primaryAxis = polygonAxis;
        } else {
            primaryAxis = edgeAxis;
        }

        var ie0 : ClipVertex = ie[0];
        var ie1 : ClipVertex = ie[1];

        if (primaryAxis.type == EPAxis.Type.EDGE_A) {
            manifold.type = Manifold.ManifoldType.FACE_A;

            // Search for the polygon normal that is most anti-parallel to the edge normal.
            var bestIndex : Int = 0;
            var bestValue : Float = Vec2.dot(m_normal, m_polygonB.normals[0]);
            for (i in 1...m_polygonB.count) {
            // for (int i = 1; i < m_polygonB.count; ++i) {
                var value : Float = Vec2.dot(m_normal, m_polygonB.normals[i]);
                if (value < bestValue) {
                    bestValue = value;
                    bestIndex = i;
                }
            }

            var i1 : Int = bestIndex;
            var i2 : Int = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;

            ie0.v.set(m_polygonB.vertices[i1]);
            ie0.id.indexA = 0;
            ie0.id.indexB = (byte) i1;
            ie0.id.typeA = (byte) ContactID.Type.FACE.ordinal();
            ie0.id.typeB = (byte) ContactID.Type.VERTEX.ordinal();

            ie1.v.set(m_polygonB.vertices[i2]);
            ie1.id.indexA = 0;
            ie1.id.indexB = (byte) i2;
            ie1.id.typeA = (byte) ContactID.Type.FACE.ordinal();
            ie1.id.typeB = (byte) ContactID.Type.VERTEX.ordinal();

            if (m_front) {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1.set(m_v1);
                rf.v2.set(m_v2);
                rf.normal.set(m_normal1);
            } else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1.set(m_v2);
                rf.v2.set(m_v1);
                rf.normal.set(m_normal1).negateLocal();
            }
        } else {
            manifold.type = Manifold.ManifoldType.FACE_B;

            ie0.v.set(m_v1);
            ie0.id.indexA = 0;
            ie0.id.indexB = (byte) primaryAxis.index;
            ie0.id.typeA = (byte) ContactID.Type.VERTEX.ordinal();
            ie0.id.typeB = (byte) ContactID.Type.FACE.ordinal();

            ie1.v.set(m_v2);
            ie1.id.indexA = 0;
            ie1.id.indexB = (byte) primaryAxis.index;
            ie1.id.typeA = (byte) ContactID.Type.VERTEX.ordinal();
            ie1.id.typeB = (byte) ContactID.Type.FACE.ordinal();

            rf.i1 = primaryAxis.index;
            rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
            rf.v1.set(m_polygonB.vertices[rf.i1]);
            rf.v2.set(m_polygonB.vertices[rf.i2]);
            rf.normal.set(m_polygonB.normals[rf.i1]);
        }

        rf.sideNormal1.set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2.set(rf.sideNormal1).negateLocal();
        rf.sideOffset1 = Vec2.dot(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = Vec2.dot(rf.sideNormal2, rf.v2);

        // Clip incident edge against extruded edge1 side edges.
        var np : Int;

        // Clip to box side 1
        np = clipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);

        if (np < Settings.maxManifoldPoints) {
            return;
        }

        // Clip to negative box side 1
        np = clipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);

        if (np < Settings.maxManifoldPoints) {
            return;
        }

        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type == EPAxis.Type.EDGE_A) {
            manifold.localNormal.set(rf.normal);
            manifold.localPoint.set(rf.v1);
        } else {
            manifold.localNormal.set(polygonB.m_normals[rf.i1]);
            manifold.localPoint.set(polygonB.m_vertices[rf.i1]);
        }

        var pointCount : Int = 0;
        for (i in 0...Settings.maxManifoldPoints) {
        // for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
            var separation : Float;

            separation = Vec2.dot(rf.normal, temp.set(clipPoints2[i].v).subLocal(rf.v1));

            if (separation <= m_radius) {
                var cp : ManifoldPoint = manifold.points[pointCount];

                if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                    Transform.mulTransToOutUnsafe(m_xf, clipPoints2[i].v, cp.localPoint);
                    cp.id.set(clipPoints2[i].id);
                } else {
                    cp.localPoint.set(clipPoints2[i].v);
                    cp.id.typeA = clipPoints2[i].id.typeB;
                    cp.id.typeB = clipPoints2[i].id.typeA;
                    cp.id.indexA = clipPoints2[i].id.indexB;
                    cp.id.indexB = clipPoints2[i].id.indexA;
                }

                ++pointCount;
            }
        }

        manifold.pointCount = pointCount;
    }


    public function computeEdgeSeparation(axis : EPAxis) : Void {
        axis.type = EPAxis.Type.EDGE_A;
        axis.index = m_front ? 0 : 1;
        axis.separation = Float.MAX_VALUE;
        var nx : Float = m_normal.x;
        var ny : Float = m_normal.y;

        for (i in 0...m_polygonB.count) {
        // for (int i = 0; i < m_polygonB.count; ++i) {
            var v : Vec2 = m_polygonB.vertices[i];
            var tempx : Float = v.x - m_v1.x;
            var tempy : Float = v.y - m_v1.y;
            var s : Float = nx * tempx + ny * tempy;
            if (s < axis.separation) {
                axis.separation = s;
            }
        }
    }

    private var perp : Vec2 = new Vec2();
    private var n : Vec2 = new Vec2();

    public function computePolygonSeparation(axis : EPAxis) : Void {
        axis.type = EPAxis.Type.UNKNOWN;
        axis.index = -1;
        axis.separation = -Float.MAX_VALUE;

        perp.x = -m_normal.y;
        perp.y = m_normal.x;

        for (i in 0...m_polygonB.count) {
        // for (int i = 0; i < m_polygonB.count; ++i) {
            var normalB : Vec2 = m_polygonB.normals[i];
            var vB : Vec2 = m_polygonB.vertices[i];
            n.x = -normalB.x;
            n.y = -normalB.y;

            var tempx : Float = vB.x - m_v1.x;
            var tempy : Float = vB.y - m_v1.y;
            var s1 : Float = n.x * tempx + n.y * tempy;
            tempx = vB.x - m_v2.x;
            tempy = vB.y - m_v2.y;
            var s2 : Float = n.x * tempx + n.y * tempy;
            var s : Float = MathUtils.min(s1, s2);

            if (s > m_radius) {
                // No collision
                axis.type = EPAxis.Type.EDGE_B;
                axis.index = i;
                axis.separation = s;
                return;
            }

            // Adjacency
            if (n.x * perp.x + n.y * perp.y >= 0.0) {
                if (Vec2.dot(temp.set(n).subLocal(m_upperLimit), m_normal) < -Settings.angularSlop) {
                continue;
                }
            } else {
                if (Vec2.dot(temp.set(n).subLocal(m_lowerLimit), m_normal) < -Settings.angularSlop) {
                continue;
                }
            }

            if (s > axis.separation) {
                axis.type = EPAxis.Type.EDGE_B;
                axis.index = i;
                axis.separation = s;
            }
        }
    }
}