using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;


public class Geometry 
{
    // Check if point is on the line or left of it.
    public static float IsAPointLeftOfVectorOrOnTheLine(Vector2 a, Vector2 b, Vector2 point)
    {
        // Determinant
        return (a.x - point.x) * (b.y - point.y) - (a.y - point.y) * (b.x - point.x);
    }

    // Clamp list indices.
    public static int ClampListIndex(int index, int listSize)
    {
	    return ((index % listSize) + listSize) % listSize;
    }

    // Check if triangle is 2D counter-clockwise or space oriented clockwise.
    public static bool IsTriangleOrientedClockwise(Vector2 p1, Vector2 p2, Vector2 p3)
    {
	    bool isClockWise = true;

	    float determinant = p1.x * p2.y + p3.x * p1.y + p2.x * p3.y - p1.x * p3.y - p3.x * p2.y - p2.x * p1.y;

	    if (determinant > 0f)
	    {
		    isClockWise = false;
	    }

	    return isClockWise;
    }

    // p1, p2, p3 are corners in the Triangle. p is the point to check for.
    public static bool IsPointInTriangle(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p)
    {
	    bool isWithinTriangle = false;

	    float denominator = ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));

	    float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / denominator;
	    float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / denominator;
	    float c = 1 - a - b;

	    // If the point is within the Triangle.
	    if (a > 0f && a < 1f && b > 0f && b < 1f && c > 0f && c < 1f)
	    {
		    isWithinTriangle = true;
	    }

	    return isWithinTriangle;
    }

    // Line segment intersection.
    public static bool AreLinesIntersecting(Vector2 l1_p1, Vector2 l1_p2, Vector2 l2_p1, Vector2 l2_p2, bool shouldIncludeEndPoints)
    {
	    bool isIntersecting = false;

	    float denominator = (l2_p2.y - l2_p1.y) * (l1_p2.x - l1_p1.x) - (l2_p2.x - l2_p1.x) * (l1_p2.y - l1_p1.y);

	    // If denominator isn't > 0, the lines are parallel.
	    if (denominator != 0f)
	    {
		    float u_a = ((l2_p2.x - l2_p1.x) * (l1_p1.y - l2_p1.y) - (l2_p2.y - l2_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;
		    float u_b = ((l1_p2.x - l1_p1.x) * (l1_p1.y - l2_p1.y) - (l1_p2.y - l1_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;

		    // Are the line segments intersecting if the end points are the same.
		    if (shouldIncludeEndPoints)
		    {
			    // Intersecting if (u_a) and (u_b) are between/exactly 0 and 1.
			    if (u_a >= 0f && u_a <= 1f && u_b >= 0f && u_b <= 1f)
			    {
				    isIntersecting = true;
			    }
		    }
		    else
		    {
			    // Intersecting if (u_a) and (u_b) are between 0 and 1.
			    if (u_a > 0f && u_a < 1f && u_b > 0f && u_b < 1f)
			    {
				    isIntersecting = true;
			    }
		    }
	    }
	    return isIntersecting;
    }

    // Triangulating a convex polygon.
    public static List<Triangle> TriangulateConvexPolygon(List<Vertex> convexHullpoints)
    {
	    List<Triangle> triangles = new List<Triangle>();

	    for (int i = 2; i < convexHullpoints.Count; i++)
	    {
		    Vertex a = convexHullpoints[0];
		    Vertex b = convexHullpoints[i - 1];
		    Vertex c = convexHullpoints[i];

		    triangles.Add(new Triangle(a, b, c));
	    }
	    return triangles;
    }

    // Check if Vertex is reflex or convex.
    private static void CheckIfReflexOrConvex(Vertex v)
    {
        // Reflex or Convex.
	    v.isReflex = false;
	    v.isConvex = false;

	    // Reflex vertex if the triangle is oriented clockwise.
	    Vector2 a = v.prevVertex.GetPos2D_XZ();
	    Vector2 b = v.GetPos2D_XZ();
	    Vector2 c = v.nextVertex.GetPos2D_XZ();

	    if (Geometry.IsTriangleOrientedClockwise(a, b, c))
	    {
		    v.isReflex = true;
	    }
	    else
	    {
		    v.isConvex = true;
	    }
    }

    // Jarvis March algorithm.
    public static List<Vertex> GetConvexHull(List<Vertex> points)
    {
        // If we have 3 points => convex hull.
        if (points.Count == 3)
        {
            return points;
        }

        // If fewer < 3 => Can't create convex hull.
        if (points.Count < 3)
        {
            return null;
        }

        // The list with points on the convex hull.
        List<Vertex> convexHull = new List<Vertex>();

        // 1. Find the vertex with the smallest x coordinate.
        Vertex startVertex = points[0];

        Vector3 startPos = startVertex.position;

        for (int i = 1; i < points.Count; i++)
        {
            Vector3 testPos = points[i].position;

            // If several have the same x coordinate, find the one with the smallest z.
            if (testPos.x < startPos.x || (Mathf.Approximately(testPos.x, startPos.x) && testPos.z < startPos.z))
            {
                startVertex = points[i];
                startPos = startVertex.position;
            }
        }

        // This vertex is always on the convex hull.
        convexHull.Add(startVertex);
        points.Remove(startVertex);

        // 2. Generate the convex hull.
        Vertex currentPoint = convexHull[0];

        List<Vertex> colinearPoints = new List<Vertex>();

        int counter = 0;

        while (true)
        {
            // Add the start position again so we can terminate the algorithm.
            if (counter == 2)
            {            
                points.Add(convexHull[0]);
            }
    
            // Random next point.
            Vertex nextPoint = points[UnityEngine.Random.Range(0, points.Count)];

            Vector2 a = currentPoint.GetPos2D_XZ();
            Vector2 b = nextPoint.GetPos2D_XZ();

            for (int i = 0; i < points.Count; i++)
            {
                if (points[i].Equals(nextPoint))
                {
                    continue;
                }
        
                Vector2 c = points[i].GetPos2D_XZ();

                float relation = Geometry.IsAPointLeftOfVectorOrOnTheLine(a, b, c);
            
                // Floating point precision issue.
                float accuracy = 0.00001f;

                if (relation < accuracy && relation > -accuracy)
                {
                    colinearPoints.Add(points[i]);
                }

                // To the right = better point = next point on the convex hull.
                else if (relation < 0f)
                {
                    nextPoint = points[i];

                    b = nextPoint.GetPos2D_XZ();

                    colinearPoints.Clear();
                }
                // To the left = worse point = do nothing.
            }

    

            // If we have colinear points.
            if (colinearPoints.Count > 0)
            {
                colinearPoints.Add(nextPoint);

                // Sort list.
                colinearPoints = colinearPoints.OrderBy(n => Vector3.SqrMagnitude(n.position - currentPoint.position)).ToList();
                convexHull.AddRange(colinearPoints);
                currentPoint = colinearPoints[colinearPoints.Count - 1];

                // Remove points that are now on the convex hull.
                for (int i = 0; i < colinearPoints.Count; i++)
                {
                    points.Remove(colinearPoints[i]);
                }

                colinearPoints.Clear();
            }
            else
            {
                convexHull.Add(nextPoint);
                points.Remove(nextPoint);
                currentPoint = nextPoint;
            }

            // If first point on the hull is found => completed the hull.
            if (currentPoint.Equals(convexHull[0]))
            {
                // Remove because it's the same as the first point => No duplicates.
                convexHull.RemoveAt(convexHull.Count - 1);

                break;
            }

            counter += 1;
        }

        return convexHull;
    }

    // Convert Triangle to Half edge.
    public static List<HalfEdge> TransformFromTriangleToHalfEdge(List<Triangle> triangles)
    {
	    // Same orientation.
	    OrientTrianglesClockwise(triangles);

	    // List with all possible half edges.
	    List<HalfEdge> halfEdges = new List<HalfEdge>(triangles.Count * 3);

	    for (int i = 0; i < triangles.Count; i++)
	    {
		    Triangle t = triangles[i];
	
		    HalfEdge he1 = new HalfEdge(t.v1);
		    HalfEdge he2 = new HalfEdge(t.v2);
		    HalfEdge he3 = new HalfEdge(t.v3);

		    he1.nextEdge = he2;
		    he2.nextEdge = he3;
		    he3.nextEdge = he1;

		    he1.prevEdge = he3;
		    he2.prevEdge = he1;
		    he3.prevEdge = he2;

		    he1.v.halfEdge = he2;
		    he2.v.halfEdge = he3;
		    he3.v.halfEdge = he1;

		    // The face.
		    t.halfEdge = he1;

		    he1.t = t;
		    he2.t = t;
		    he3.t = t;

		    halfEdges.Add(he1);
		    halfEdges.Add(he2);
		    halfEdges.Add(he3);
	    }

	    // The half edges going in the opposite direction.
	    for (int i = 0; i < halfEdges.Count; i++)
	    {
		    HalfEdge he = halfEdges[i];

		    Vertex goingToVertex = he.v;
		    Vertex goingFromVertex = he.prevEdge.v;

		    for (int j = 0; j < halfEdges.Count; j++)
		    {
			    // Don't compare to itself.
			    if (i == j)
			    {
				    continue;
			    }

			    HalfEdge heOpposite = halfEdges[j];

			    // If edge is going between the vertices in the opposite direction.
			    if (goingFromVertex.position == heOpposite.v.position && goingToVertex.position == heOpposite.prevEdge.v.position)
			    {
				    he.oppositeEdge = heOpposite;
				    break;
			    }
		    }
	    }
	    return halfEdges;
    }

    // Orient triangles for correct orientation.
    public static void OrientTrianglesClockwise(List<Triangle> triangles)
    {
	    for (int i = 0; i < triangles.Count; i++)
	    {
		    Triangle tri = triangles[i];

		    Vector2 v1 = new Vector2(tri.v1.position.x, tri.v1.position.z);
		    Vector2 v2 = new Vector2(tri.v2.position.x, tri.v2.position.z);
		    Vector2 v3 = new Vector2(tri.v3.position.x, tri.v3.position.z);

		    if (!IsTriangleOrientedClockwise(v1, v2, v3))
		    {
			    tri.ChangeOrientation();
		    }
	    }
    }

    // Positive if inside, negative if outside and 0 if on the circle.
    public static float IsPointInsideOutsideOrOnCircle(Vector2 aVec, Vector2 bVec, Vector2 cVec, Vector2 dVec)
    {
        // Simplify the determinant calculation.
	    float a = aVec.x - dVec.x;
	    float d = bVec.x - dVec.x;
	    float g = cVec.x - dVec.x;

	    float b = aVec.y - dVec.y;
	    float e = bVec.y - dVec.y;
	    float h = cVec.y - dVec.y;

	    float c = a * a + b * b;
	    float f = d * d + e * e;
	    float i = g * g + h * h;

	    float determinant = (a * e * i) + (b * f * g) + (c * d * h) - (g * e * c) - (h * f * a) - (i * d * b);

	    return determinant;
    }

    // Check if it's a Quadrilateral convex.
    public static bool IsQuadrilateralConvex(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
	    bool isConvex = false;

	    bool abc = IsTriangleOrientedClockwise(a, b, c);
	    bool abd = IsTriangleOrientedClockwise(a, b, d);
	    bool bcd = IsTriangleOrientedClockwise(b, c, d);
	    bool cad = IsTriangleOrientedClockwise(c, a, d);

	    if (abc && abd && bcd & !cad)
	    {
		    isConvex = true;
	    }
	    else if (abc && abd && !bcd & cad)
	    {
		    isConvex = true;
	    }
	    else if (abc && !abd && bcd & cad)
	    {
		    isConvex = true;
	    }
	    // Opposite sign => everything inverted.
	    else if (!abc && !abd && !bcd & cad)
	    {
		    isConvex = true;
	    }
	    else if (!abc && !abd && bcd & !cad)
	    {
		    isConvex = true;
	    }
	    else if (!abc && abd && !bcd & !cad)
	    {
		    isConvex = true;
	    }

	    return isConvex;
    }

    // Triangulate and then flip edges until we have a Delaunay triangulation.
    public static List<Triangle> TriangulateByFlippingEdges(List<Vector3> sites)
    {
	    // 1. Triangulate the points.

	    // Vector3 => Vertex.
	    List<Vertex> vertices = new List<Vertex>();
	    for (int i = 0; i < sites.Count; i++)
	    {
		    vertices.Add(new Vertex(sites[i]));
	    }

	    // Triangulate the convex hull of the sites.
	    List<Triangle> triangles = TriangulatePoints(vertices);

        // 2. Convert triangles to half-edges, faster to flip edges.
	    List<HalfEdge> halfEdges = TransformFromTriangleToHalfEdge(triangles);

        // 3. Flip edges until we get Delaunay triangulation.
	    int safety = 0;
	    int flippedEdges = 0;

	    while (true)
	    {
		    safety += 1;

		    if (safety > 100000)
		    {
			    Debug.Log("[TriangulateByFlippingEdges] - Endless loop.");
			    break;
		    }

		    bool hasFlippedEdge = false;

		    // Check if we can flip an edge.
		    for (int i = 0; i < halfEdges.Count; i++)
		    {
			    HalfEdge thisEdge = halfEdges[i];

			    // If this edge is sharing an edge else it's a border and then we can't flip the edge.
			    if (thisEdge.oppositeEdge == null)
			    {
				    continue;
			    }

			    Vertex a = thisEdge.v;
			    Vertex b = thisEdge.nextEdge.v;
			    Vertex c = thisEdge.prevEdge.v;
			    Vertex d = thisEdge.oppositeEdge.nextEdge.v;

			    Vector2 aPos = a.GetPos2D_XZ();
			    Vector2 bPos = b.GetPos2D_XZ();
			    Vector2 cPos = c.GetPos2D_XZ();
			    Vector2 dPos = d.GetPos2D_XZ();

                // IsPointInsideOutsideOrOnCircle if we need to flip this edge.
                if (IsPointInsideOutsideOrOnCircle(aPos, bPos, cPos, dPos) < 0f)
			    {
				    // If the two triangles that share this edge is forming a convex quadrilateral, If not => can't be flipped.
				    if (IsQuadrilateralConvex(aPos, bPos, cPos, dPos))
				    {
					    // If new triangle after a flip is not better, then don't flip.
					    if (IsPointInsideOutsideOrOnCircle(bPos, cPos, dPos, aPos) < 0f)
					    {
						    continue;
					    }

					    flippedEdges += 1;
					    hasFlippedEdge = true;
					    FlipEdge(thisEdge);
				    }
			    }
		    }

		    
		    if (!hasFlippedEdge)
		    {
                // When we have gone through all edges and havent found an edge to flip => Delaunay triangulation.
                break;
		    }
	    }
	    return triangles;
    }

    // Flip edge.
    private static void FlipEdge(HalfEdge one)
    {
	    // The triangle.
	    HalfEdge two = one.nextEdge;
	    HalfEdge three = one.prevEdge;

	    // The opposite edges triangle.
	    HalfEdge four = one.oppositeEdge;
	    HalfEdge five = one.oppositeEdge.nextEdge;
	    HalfEdge six = one.oppositeEdge.prevEdge;

	    // Vertices
	    Vertex a = one.v;
	    Vertex b = one.nextEdge.v;
	    Vertex c = one.prevEdge.v;
	    Vertex d = one.oppositeEdge.nextEdge.v;

	    // 1. Flip

	    // Change Vertex.
	    a.halfEdge = one.nextEdge;
	    c.halfEdge = one.oppositeEdge.nextEdge;

	    // Change half-edge.
	    one.nextEdge = three;
	    one.prevEdge = five;

	    two.nextEdge = four;
	    two.prevEdge = six;

	    three.nextEdge = five;
	    three.prevEdge = one;

	    four.nextEdge = six;
	    four.prevEdge = two;

	    five.nextEdge = one;
	    five.prevEdge = three;

	    six.nextEdge = two;
	    six.prevEdge = four;

	    one.v = b;
	    two.v = b;
	    three.v = c;
	    four.v = d;
	    five.v = d;
	    six.v = a;

	    Triangle t1 = one.t;
	    Triangle t2 = four.t;

	    one.t = t1;
	    three.t = t1;
	    five.t = t1;

	    two.t = t2;
	    four.t = t2;
	    six.t = t2;

	    // Triangle connection.
	    t1.v1 = b;
	    t1.v2 = c;
	    t1.v3 = d;

	    t2.v1 = b;
	    t2.v2 = d;
	    t2.v3 = a;

	    t1.halfEdge = three;
	    t2.halfEdge = four;
    }

    // Triangulate the convex hull of the sites.
    public static List<Triangle> TriangulatePoints(List<Vertex> points)
    {
        List<Triangle> triangles = new List<Triangle>();

        // Sort the points along x-axis.
        points = points.OrderBy(n => n.position.x).ToList();

        // Triangle.
        Triangle newTriangle = new Triangle(points[0].position, points[1].position, points[2].position);
    
        triangles.Add(newTriangle);

        List<Edge> edges = new List<Edge>();

        edges.Add(new Edge(newTriangle.v1, newTriangle.v2));
        edges.Add(new Edge(newTriangle.v2, newTriangle.v3));
        edges.Add(new Edge(newTriangle.v3, newTriangle.v1));

        // Add the other triangles. Start at 3 because we already added the first.
        for (int i = 3; i < points.Count; i++)
        {
            Vector3 currentPoint = points[i].position;

            List<Edge> newEdges = new List<Edge>();

            // Is this edge visible?
            for (int j = 0; j < edges.Count; j++)
            {
                Edge currentEdge = edges[j];

                Vector3 midPoint = (currentEdge.v1.position + currentEdge.v2.position) / 2f;

                Edge edgeToMidpoint = new Edge(currentPoint, midPoint);

                // If this line is intersecting.
                bool canSeeEdge = true;

                for (int k = 0; k < edges.Count; k++)
                {
                    // Don't compare with itself.
                    if (k == j)
                    {
                        continue;
                    }

                    if (AreEdgesIntersecting(edgeToMidpoint, edges[k]))
                    {
                        canSeeEdge = false;

                        break;
                    }
                }

                // Valid triangle.
                if (canSeeEdge)
                {
                    Edge edgeToPoint1 = new Edge(currentEdge.v1, new Vertex(currentPoint));
                    Edge edgeToPoint2 = new Edge(currentEdge.v2, new Vertex(currentPoint));

                    newEdges.Add(edgeToPoint1);
                    newEdges.Add(edgeToPoint2);

                    Triangle newTri = new Triangle(edgeToPoint1.v1, edgeToPoint1.v2, edgeToPoint2.v1);

                    triangles.Add(newTri);
                }
            }

            for (int j = 0; j < newEdges.Count; j++)
            {
                edges.Add(newEdges[j]);
            }
        }


        return triangles;
    }

    // Check if edges are intersecting.
    private static bool AreEdgesIntersecting(Edge edge1, Edge edge2)
    {
        Vector2 l1_p1 = new Vector2(edge1.v1.position.x, edge1.v1.position.z);
        Vector2 l1_p2 = new Vector2(edge1.v2.position.x, edge1.v2.position.z);
    
        Vector2 l2_p1 = new Vector2(edge2.v1.position.x, edge2.v1.position.z);
        Vector2 l2_p2 = new Vector2(edge2.v2.position.x, edge2.v2.position.z);

        return AreLinesIntersecting(l1_p1, l1_p2, l2_p1, l2_p2, true);
    }
}
