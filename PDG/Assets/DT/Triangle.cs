using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Triangle class.
public class Triangle
{
    // Corners
    public Vertex v1;
    public Vertex v2;
    public Vertex v3;

    public float v1_v2;
    public float v1_v3;
    public float v2_v1;
    public float v2_v3;
    public float v3_v1;
    public float v3_v2;

    // Colors, if we want to render the triangle.
    float color1 = Random.value;
    float color2 = Random.value;
    float color3 = Random.value;

    public HalfEdge halfEdge;

    // Calculate the weights based on distance.
    public void calcWeights()
    {
        v1_v2 = Mathf.Round(Vector3.Distance(v1.position, v2.position));
        v1_v3 = Mathf.Round(Vector3.Distance(v1.position, v3.position));

        v2_v1 = Mathf.Round(Vector3.Distance(v2.position, v1.position));
        v2_v3 = Mathf.Round(Vector3.Distance(v2.position, v3.position));

        v3_v1 = Mathf.Round(Vector3.Distance(v3.position, v1.position));
        v3_v2 = Mathf.Round(Vector3.Distance(v3.position, v2.position));
    }

    // Constructor.
    public Triangle(Vertex v1, Vertex v2, Vertex v3)
    {
        this.v1 = v1;
        this.v2 = v2;
        this.v3 = v3;
    }

    // Constructor.
    public Triangle(Vector3 v1, Vector3 v2, Vector3 v3)
    {
        this.v1 = new Vertex(v1);
        this.v2 = new Vertex(v2);
        this.v3 = new Vertex(v3);
    }

    // Constructor.
    public Triangle(HalfEdge halfEdge)
    {
        this.halfEdge = halfEdge;
    }

    // Change Triangles orientation.
    // cw -> ccw or ccw -> cw
    public void ChangeOrientation()
    {
        Vertex temp = this.v1;

        this.v1 = this.v2;
        this.v2 = temp;
    }

    // Draw the triangle.
    public void draw()
    {
		Material material = new Material(Shader.Find("Sprites/Default"));
		material.color = new Color(color1,color2,color3,0.7f);
		material.SetPass(0);

		GL.PushMatrix();
		GL.Begin(GL.TRIANGLES);
		GL.Vertex3(v1.position.x, v1.position.y+1, v1.position.z);
		GL.Vertex3(v2.position.x, v2.position.y+1, v2.position.z);
		GL.Vertex3(v3.position.x, v3.position.y+1, v3.position.z);
		GL.End();
		GL.PopMatrix();
    }
}