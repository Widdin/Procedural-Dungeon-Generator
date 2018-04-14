using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Vertex class.
public class Vertex
{
    // The position of the vertex.
    public Vector3 position;

    // A HalfEdge that starts at this vertex.
    public HalfEdge halfEdge;

    // Vertex is part of this Triangle.
    public Triangle triangle;

    // The previous/next Vertex this vertex is connected to.
    public Vertex prevVertex;
    public Vertex nextVertex;

    // Properties.
    public bool isReflex; 
    public bool isConvex;
    public bool isEar;

    // Constructor with Vector3.
    public Vertex(Vector3 position)
    {
        this.position = position;
    }

    // Return 2D position.
    public Vector2 GetPos2D_XZ()
    {
        return new Vector2(position.x, position.z);
    }
}