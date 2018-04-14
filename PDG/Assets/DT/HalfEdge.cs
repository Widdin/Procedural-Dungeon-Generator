using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// HalfEdge class.
public class HalfEdge
{
    // HalfEdge points to this Vertex.
    public Vertex v;

    // HalfEdge is part of this face.
    public Triangle t;

    // The previous/next edge.
    public HalfEdge prevEdge;
    public HalfEdge nextEdge;

    // The edge with opposite direction.
    public HalfEdge oppositeEdge;

    // Constructor.
    public HalfEdge(Vertex v)
    {
        this.v = v;
    }
}