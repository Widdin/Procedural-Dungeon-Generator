using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class seperation : MonoBehaviour {

	public Rect pos;

	public bool isMainRoom = false;
	public bool isHall = false;
	public bool disable = false;

	public int tileSize = 1;

	private GameObject[] gO;
	public int strength = 1;
	public int bugg = 0;

    public bool update = true;

	float roundm(float n, float m)
	{
		return Mathf.Floor((n + m - 1) /m)*m;
	}

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () 
	{
        if(update)
        {
		    gO = GameObject.FindGameObjectsWithTag ("pform");
		    foreach (GameObject a in gO) 
		    {
			    if (a != this) 
			    {
				    if (this.GetComponent<Renderer> ().bounds.Intersects (a.GetComponent<Renderer> ().bounds)) 
				    {
					    Vector3 direction = transform.localPosition - a.transform.localPosition;
					    direction.Normalize ();
					    transform.localPosition = new Vector3 (roundm(transform.localPosition.x + (direction.x * strength), tileSize), transform.localPosition.y, roundm(transform.localPosition.z + (direction.z * strength), tileSize));
				    }
			    }
		    }
        }


        if (isMainRoom)
		{
			Renderer rend = this.GetComponent<Renderer>();
			rend.material.color = new Color(1,0,0,1);
		}

	}
}
