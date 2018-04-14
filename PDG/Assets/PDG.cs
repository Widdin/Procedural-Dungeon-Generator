using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;
using System;

public class PDG : MonoBehaviour
{
    // Platforms.
	public GameObject       platform;
	public List<GameObject> platforms;

    // Total number of rooms.
	public int numberOfRooms;
	public int tileSize;

    // Room containers.
	public List<Rect>       allRooms;
	public List<GameObject> mainRooms;

    //Delaunay
	public static List<Triangle> delaunay;

    // Room variables.
	public int RoomMaxWidth = 30;
	public int RoomMinWidth = 9;
	public int RoomMaxHeight = 30;
	public int RoomMinHeight = 9;

    public float MainRoomLimit = 0.75f;
    public float RoomRandomCircleRadius = 60.0f;

    // Prims - Minimum spanning tree.
    public List<edge> priorityQueue;
    public List<edge> MST;
    public List<Vector3> treeVertices;
    public static List<edge> treeEdges;

    // Hallway edges.
    public static List<edge> hallwayGraph;

    // Camera
    GameObject Cam;

    void Start () 
	{
        // Set nbr of rooms according to slider.
        numberOfRooms = (int)GameObject.Find("roomSlider").GetComponent <Slider> ().value;

        // Create empty lists.
        delaunay = new List<Triangle>();
        hallwayGraph  = new List<edge>();

        // Find the main camera.
        Cam = GameObject.Find("Main Camera");

        spawnRooms();

        Time.timeScale = 10;
		StartCoroutine (drawRooms());
	}


	void Update()
	{
        // Update text based on slider.
        GameObject.Find("roomSliderText").GetComponent <Text> ().text = GameObject.Find("roomSlider").GetComponent <Slider> ().value.ToString();

		if (Input.GetKeyDown(KeyCode.Alpha1))
		{
			findMainRooms();
		}
		if (Input.GetKeyDown(KeyCode.Alpha2))
		{
			generateGraph();
		}
        if(Input.GetKeyDown(KeyCode.Alpha3))
        {
            generateHallways();
        }
	}

	float roundm(float n, float m)
	{
		return Mathf.Floor((n + m - 1) /m) * m;
	}

    // Return a random point (x,y) inside a circles radius.
	Vector2 getRandomPointInCircle(float radius)
	{
		float t = 2 * Mathf.PI * UnityEngine.Random.value;
		float u = UnityEngine.Random.value + UnityEngine.Random.value;
		float r = 0.0f;

		if(u > 1)
        {
            r = 2 - u;
        }
		else
        {
            r = u;
        }

		Vector2 result = new Vector2(roundm(radius * r * Mathf.Cos(t) , tileSize), roundm(radius * r * Mathf.Sin(t) , tileSize));
		return result;
	}

    // Normal/Gaussian distribution.
	float NextGaussianDouble(float mean, float deviation)
	{
        float u, v, S;

        do
        {
            u = 2.0f * UnityEngine.Random.value - 1.0f;
            v = 2.0f * UnityEngine.Random.value - 1.0f;
            S = u * u + v * v;
        }
        while (S >= 1.0);

	    float fac = Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);
	    return deviation * (u * fac) + mean;
	}

    // Store a Rect with a random width/height inside 'allRooms'.
	void spawnRooms()
	{
        print("Started - Spawning rooms...");
		allRooms = new List<Rect>();
		for (int i = 0; i < numberOfRooms; i ++)
        {
			Vector2 r = getRandomPointInCircle (RoomRandomCircleRadius);
			Rect room = new Rect (r.x + RoomRandomCircleRadius,
			                     r.y + RoomRandomCircleRadius,
			                     Mathf.Round (UnityEngine.Random.Range (RoomMinWidth, RoomMaxWidth)),
			                     Mathf.Round (UnityEngine.Random.Range (RoomMinHeight, RoomMaxHeight))
			);
			allRooms.Add (room);
		}

        print("Finished - Spawning rooms.");
        print(numberOfRooms + " rooms were spawned.");
    }

    // Instantiate the rooms.
	IEnumerator drawRooms()
	{

		for(int i = 0; i < allRooms.Count; i++)
		{
			GameObject prefab = Instantiate(platform) as GameObject;
			prefab.transform.parent = transform;
			prefab.name = "Rect Nr: " + i;
			prefab.transform.localPosition = new Vector3(allRooms[i].x, 1.0f, allRooms[i].y);
			prefab.transform.localScale = new Vector3(allRooms[i].width, 1.0f, allRooms[i].height);

			platforms.Add(prefab);
            yield return new WaitForSeconds(0.5f);
		}
        StartCoroutine(findMainRooms());
	}

    // Find rooms based on a limit.
	IEnumerator findMainRooms()
	{
        print("Started - Finding main rooms...");

        // Bounds needed to count as a main room.
		float minWidth = RoomMaxWidth * MainRoomLimit;
		float minHeight = RoomMaxHeight * MainRoomLimit;

        // Find all the platforms.
		GameObject[] gameObjects = GameObject.FindGameObjectsWithTag ("pform");

        int i = 0;
        foreach (GameObject GO in gameObjects) 
		{
			Renderer renderer = GO.GetComponent<Renderer>();

			float x = renderer.bounds.size.x;
			float y = renderer.bounds.size.z;

            var script = GO.GetComponent<seperation>();
            script.update = false;

            if (x > minHeight && y > minWidth) 
			{
                // Color it red.
                renderer.material.color = new Color(1,0,0,1);
				GO.name = "Main room: " + i;
                // Add it to 'mainRooms'.
				mainRooms.Add(GO);
                i++;

                yield return new WaitForSeconds(0.5f);
            }
		}

        print("Finished - Finding main rooms.");
        print("Total main rooms: " + i);
        print("Out of " + allRooms.Count + " total rooms.");

        StartCoroutine(generateGraph());
    }

    // Generate delaunay graph.
	IEnumerator generateGraph()
	{
        print("Started - Generating Delaunay graph...");

        List<Vector3> vectorMainPos =   new List<Vector3>();
        List<Vertex> vertexMainPos =    new List<Vertex>();

        List<edge> nodes =              new List<edge>();
        List<Vertex> convexHullPoints = new List<Vertex>();
        List<Triangle> convexPolygon =  new List<Triangle>();

        delaunay = new List<Triangle>();

        // Store all main room positions in vector and vertex container format.
        foreach (GameObject GO in mainRooms)
		{
			Vertex vertPos = new Vertex(GO.transform.position);
            Vector3 vecPos = new Vector3(GO.transform.position.x, GO.transform.position.y, GO.transform.position.z);
            vertexMainPos.Add(vertPos);
            vectorMainPos.Add(vecPos);
        }

        // Generate convex hullpoints.
        convexHullPoints = Geometry.GetConvexHull(vertexMainPos);

        // Generate convex polygon.
		convexPolygon = Geometry.TriangulateConvexPolygon(convexHullPoints);

        delaunay = Geometry.TriangulateByFlippingEdges(vectorMainPos);

		foreach(Triangle triangle in delaunay)
		{
			nodes.Add(new edge(triangle.v1.position, triangle.v2.position, Mathf.Round(Vector3.Distance(triangle.v1.position, triangle.v2.position))));
			nodes.Add(new edge(triangle.v2.position, triangle.v3.position, Mathf.Round(Vector3.Distance(triangle.v2.position, triangle.v3.position))));
			nodes.Add(new edge(triangle.v3.position, triangle.v1.position, Mathf.Round(Vector3.Distance(triangle.v3.position, triangle.v1.position))));
            yield return new WaitForSeconds(0.5f);
		}

        // Delete duplicates.
		for(int i = 0; i < nodes.Count; i++)
		{
			foreach(edge b in nodes.ToList())
			{
				if(nodes[i].start.Equals(b.end) && b.end.Equals(nodes[i].start) && nodes[i].weight == b.weight)
				{
					nodes.Remove(b);	
				}
			}
		}

        print("Finished - Generating Delaunay graph.");
        Cam.GetComponent<cameraRender>().toggleDelaunay = true;
        // Prims algorithm on the resulting graph from Delaunay.
        StartCoroutine(prims(nodes));
    }


    [System.Serializable]
	public class edge
	{
		public Vector3 start; 	// Starting vertex (node) of the edge
		public Vector3 end;		// Ending vertex (node) of the edge

		public float weight; 	// Weight of the edge

		public edge(Vector3 start, Vector3 end, float weight)
		{
			this.start = start;
			this.end = end;

			this.weight = weight;
		}

	}

    // Prims algorithm (Minimum spanning tree):
    IEnumerator prims(List<edge> vert)
    {
        print("Started - Prims Algorithm...");

        // Copy the vertices.
        List<edge> edges = vert;
        List<Vector3> vertices = new List<Vector3>();

        treeVertices = new List<Vector3>();
        treeEdges = new List<edge>();

        for (int i = 0; i < edges.Count; i++)
        {
            if(!vertices.Contains(edges[i].start))
            {
                vertices.Add(edges[i].start);
            }
            else if(!vertices.Contains(edges[i].end))
            {
                vertices.Add(edges[i].end);
            }
        }

        int randomVertice = UnityEngine.Random.Range(0,vertices.Count); 

        treeVertices.Add(vertices[randomVertice]);
        vertices.RemoveAt(randomVertice);

        while (vertices.Count > 0)
        {
            edge optEdge = null;

            foreach (edge e in edges)
            {
                if ((vertices.Contains(e.start) && !vertices.Contains(e.end)) ||
                  (vertices.Contains(e.end) && !vertices.Contains(e.start)))
                {
                    if (optEdge == null)
                    {
                        optEdge = e;
                    }
                    else if(optEdge.weight > e.weight)
                    {
                        optEdge = e;
                    }
                    
                }
            }

            Vector3 vertex = new Vector3();

            if(vertices.Contains(optEdge.start))
            {
                vertex = optEdge.start;
            }
            else
            {
                vertex = optEdge.end;
            }

            vertices.Remove(vertex);
            treeVertices.Add(vertex);

            edges.Remove(optEdge);
            treeEdges.Add(optEdge);

            yield return new WaitForSeconds(1.0f);
        }
        print("Finished - Prims algorithm.");

        Cam.GetComponent<cameraRender>().toggleDelaunay = false;
        Cam.GetComponent<cameraRender>().togglePrims = true;

        StartCoroutine(generateHallways());
    }

    // Draw the graph that Prim has generated.
    public static IEnumerator drawPrim()
    {
            for (int i = 0; i < treeEdges.Count; i++)
            {
                Material material = new Material(Shader.Find("Sprites/Default"));
                material.color = Color.blue;
                material.SetPass(0);

                GL.PushMatrix();
                GL.Begin(GL.LINES);
                GL.Vertex3(treeEdges[i].start.x, -1, treeEdges[i].start.z);
                GL.Vertex3(treeEdges[i].end.x, -1, treeEdges[i].end.z);
                GL.End();
                GL.PopMatrix();

                yield return new WaitForEndOfFrame();
            }
    }

    // Draw the lines that 'generateHallways' has generated.
    public static IEnumerator drawHallways()
    {
            for (int i = 0; i < hallwayGraph.Count; i++)
            {
                Material material = new Material(Shader.Find("Sprites/Default"));
                material.color = Color.green;
                material.SetPass(0);

                GL.PushMatrix();
                GL.Begin(GL.LINES);
                GL.Vertex3(hallwayGraph[i].start.x, -1, hallwayGraph[i].start.z);
                GL.Vertex3(hallwayGraph[i].end.x, -1, hallwayGraph[i].end.z);
                GL.End();
                GL.PopMatrix();

                yield return new WaitForEndOfFrame();
            }
    }

    // Generate the hallways.
    public IEnumerator generateHallways()
    {
        print("Started - Generating hallways...");
        hallwayGraph = new List<edge>();

        foreach (edge edge in treeEdges)
        {
            GameObject a = null;
            GameObject b = null;

            foreach(GameObject rect in mainRooms)
            {
                Renderer rend = rect.GetComponent<Renderer>();

                if (a == null && rend.bounds.Contains(edge.start))
                {
                    a = rect;
                }
                else if (b == null && rend.bounds.Contains(edge.end))
                {
                    b = rect;
                }

                if (a != null && b != null)
                {
                    break;
                }

            }

            if (a == null || b == null)
            {
                print("Unable to find all main regions for edge, skipping edge");
                continue;
            }

            Renderer aRend = a.GetComponent<Renderer>();
            Renderer bRend = b.GetComponent<Renderer>();

            Vector2 aCenter = new Vector2(aRend.bounds.center.x, aRend.bounds.center.z);
            Vector2 bCenter = new Vector2(bRend.bounds.center.x, bRend.bounds.center.z);

            Vector2 midpoint = (aCenter + bCenter) / 2;

            bool useX = false;
            if(midpoint.x >= aRend.bounds.min.x && midpoint.x <= aRend.bounds.max.x &&
                midpoint.x >= bRend.bounds.min.x && midpoint.x <= bRend.bounds.max.x)
            {
                useX = true;
            }

            bool useY = false;
            if (midpoint.y >= aRend.bounds.min.y && midpoint.y <= aRend.bounds.max.y &&
                midpoint.y >= bRend.bounds.min.y && midpoint.y <= bRend.bounds.max.y)
            {
                useY = true;
            }

            if (useX && useY)
            {
                // 50% Chance of being either useX or useY
                bool Boolean = (UnityEngine.Random.value > 0.5f);

                if (Boolean)
                {
                    useX = false;
                }
                else
                {
                    useY = false;
                }
            }

            if (useX)
            {
                // Hallway goes up/down
                hallwayGraph.Add(new edge(
                    new Vector3(midpoint.x, 1.0f, aCenter.y),
                    new Vector3(midpoint.x, 1.0f, bCenter.y),
                    1.0f
                    ));
            }
            else if (useY)
            {
                // Hallway goes left/right
                hallwayGraph.Add(new edge(
                    new Vector3(aCenter.x, 1.0f, midpoint.y),
                    new Vector3(bCenter.x, 1.0f, midpoint.y),
                    1.0f
                    ));
            }
            else
            {
                // Hallway has a corner
                hallwayGraph.Add(new edge(
                    new Vector3(aCenter.x, 1.0f, aCenter.y),
                    new Vector3(aCenter.x, 1.0f, bCenter.y),
                    1.0f
                    ));
                hallwayGraph.Add(new edge(
                    new Vector3(aCenter.x, 1.0f, bCenter.y),
                    new Vector3(bCenter.x, 1.0f, bCenter.y),
                    1.0f
                    ));
            }

            yield return new WaitForSeconds(1.0f);
        }
        print("Finished - Generating hallways.");

        Cam.GetComponent<cameraRender>().togglePrims = false;
        Cam.GetComponent<cameraRender>().toggleHallways = true;

        StartCoroutine(deleteNonMainRooms());
    }

    // Delete all the white colored rooms, a.k.a not main rooms.
    public IEnumerator deleteNonMainRooms()
    {
        for(int i = 0; i < platforms.Count; i++)
        {
            if(!mainRooms.Contains(platforms[i]))
            {
                Destroy(platforms[i]);
                yield return new WaitForSeconds(0.05f);
            }
        }

        yield return new WaitForSeconds(10f);

        // Restart the scene.
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

}
